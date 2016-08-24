/*
*  drivers/cpufreq/cpufreq_smartactivev2.c
*  Auther : NewWorld(cae11cae@naver.com)
*  based on governor Conservative, Authers :
*
*  Copyright (C)  2001 Russell King
*            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
*                      Jun Nakajima <jun.nakajima@intel.com>
*            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
* 
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/earlysuspend.h>

/* On / off debug */
/* This function does not use now, but who knows.. */
//#define SMARTACTIVEV2_DEBUG
#undef SMARTACTIVEV2_DEBUG
/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */
#define DEF_AWAKE_UP_THRESHOLD			(80)
#define DEF_AWAKE_DOWN_THRESHOLD		(20)
#define DEF_AWAKE_UP_FREQ_STEP			(1)
#define DEF_AWAKE_DOWN_FREQ_STEP		(2)
#define DEF_ASLEEP_UP_THRESHOLD		(90)
#define DEF_ASLEEP_DOWN_THRESHOLD		(30)
#define DEF_ASLEEP_UP_FREQ_STEP		(1)
#define DEF_ASLEEP_DOWN_FREQ_STEP		(5)
#define DEF_SAMPLING_RATE			(40000)

#define MIN_SAMPLING_RATE			(10000)
/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(10)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

static unsigned int is_early_suspend = 0;
int up_threshold = DEF_AWAKE_UP_THRESHOLD;
int down_threshold = DEF_AWAKE_DOWN_THRESHOLD;
int up_freq_step = DEF_AWAKE_UP_FREQ_STEP;
int down_freq_step = DEF_AWAKE_DOWN_FREQ_STEP;

static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int down_skip;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int sampling_down_factor;
	unsigned int ignore_nice;
	unsigned int up_threshold_awake;
	unsigned int down_threshold_awake;
	unsigned int up_threshold_asleep;
	unsigned int down_threshold_asleep;
	unsigned int up_freq_step_awake;
	unsigned int down_freq_step_awake;
	unsigned int up_freq_step_asleep;
	unsigned int down_freq_step_asleep;
} dbs_tuners_ins = {
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.ignore_nice = 0,
	.up_freq_step_awake = DEF_AWAKE_UP_FREQ_STEP,
	.down_freq_step_awake = DEF_AWAKE_DOWN_FREQ_STEP,
	.up_freq_step_asleep = DEF_ASLEEP_UP_FREQ_STEP,
	.down_freq_step_asleep = DEF_ASLEEP_DOWN_FREQ_STEP,
	.up_threshold_awake = DEF_AWAKE_UP_THRESHOLD,
	.down_threshold_awake = DEF_AWAKE_DOWN_THRESHOLD,
	.up_threshold_asleep = DEF_ASLEEP_UP_THRESHOLD,
	.down_threshold_asleep = DEF_ASLEEP_DOWN_THRESHOLD,
};


static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}
/* keep track of frequency transitions */
static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	*/
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};


/************************** sysfs interface ************************/

/* cpufreq_smartactivev2 Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(up_threshold_awake, up_threshold_awake);
show_one(down_threshold_awake, down_threshold_awake);
show_one(up_threshold_asleep, up_threshold_asleep);
show_one(down_threshold_asleep, down_threshold_asleep);
show_one(up_freq_step_awake, up_freq_step_awake);
show_one(down_freq_step_awake, down_freq_step_awake);
show_one(up_freq_step_asleep, up_freq_step_asleep);
show_one(down_freq_step_asleep, down_freq_step_asleep);

static ssize_t store_sampling_down_factor(struct kobject *a,
					  struct attribute *b,
					  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_down_factor = input;
	return count;
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;
	else if(input < MIN_SAMPLING_RATE)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate = input;
	return count;
}
static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) /* nothing to do */
		return count;

	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}
				
static ssize_t store_up_freq_step_awake					
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;					
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1)
		return -EINVAL;				
	dbs_tuners_ins.up_freq_step_awake = input;	
	return count;					
}
static ssize_t store_up_freq_step_asleep					
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;					
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1)
		return -EINVAL;				
	dbs_tuners_ins.up_freq_step_asleep = input;	
	return count;					
}
static ssize_t store_down_freq_step_awake					
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;					
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1)
		return -EINVAL;				
	dbs_tuners_ins.down_freq_step_awake = input;	
	return count;					
}
static ssize_t store_down_freq_step_asleep					
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;					
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1)
		return -EINVAL;				
	dbs_tuners_ins.down_freq_step_asleep = input;	
	return count;					
}


static ssize_t store_up_threshold_awake	
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;			
	int ret;						
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1 ||	
	dbs_tuners_ins.down_threshold_awake < input)
		return -EINVAL;				
	dbs_tuners_ins.up_threshold_awake = input;	
	return count;					
}
static ssize_t store_up_threshold_asleep	
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;						
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1 ||	
	dbs_tuners_ins.down_threshold_asleep > input)
		return -EINVAL;				
	dbs_tuners_ins.up_threshold_asleep = input;	
	return count;					
}
static ssize_t store_down_threshold_awake	
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;						
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1 ||	
	dbs_tuners_ins.up_threshold_awake < input)
		return -EINVAL;				
	dbs_tuners_ins.down_threshold_awake = input;	
	return count;					
}
static ssize_t store_down_threshold_asleep	
(struct kobject *a, struct attribute *b, const char *buf, size_t count)				
{								
	unsigned int input;				
	int ret;						
	ret = sscanf(buf, "%u", &input);		
	if(ret != 1 || input > 100 || input < 1 ||	
	dbs_tuners_ins.up_threshold_asleep < input)
		return -EINVAL;				
	dbs_tuners_ins.down_threshold_asleep = input;	
	return count;					
}

define_one_global_rw(sampling_rate);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(up_threshold_awake);
define_one_global_rw(down_threshold_awake);
define_one_global_rw(up_threshold_asleep);
define_one_global_rw(down_threshold_asleep);
define_one_global_rw(up_freq_step_awake);
define_one_global_rw(down_freq_step_awake);
define_one_global_rw(up_freq_step_asleep);
define_one_global_rw(down_freq_step_asleep);

static struct attribute *dbs_attributes[] = {
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&up_threshold_awake.attr,
	&down_threshold_awake.attr,
	&up_freq_step_awake.attr,
	&down_freq_step_awake.attr,
	&up_threshold_asleep.attr,
	&down_threshold_asleep.attr,
	&up_freq_step_asleep.attr,
	&down_freq_step_asleep.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "smartactivev2",
};


/************************** sysfs end ************************/

static void cpufreq_smartactivev2_early_suspend(struct early_suspend *h)
{
	mutex_lock(&dbs_mutex);
	up_threshold = dbs_tuners_ins.up_threshold_asleep;
	down_threshold = dbs_tuners_ins.down_threshold_asleep;
	up_freq_step = dbs_tuners_ins.up_freq_step_asleep;
	down_freq_step = dbs_tuners_ins.down_freq_step_asleep;
	is_early_suspend = 1;
	mutex_unlock(&dbs_mutex);
}

static void cpufreq_smartactivev2_late_resume(struct early_suspend *h)
{
	mutex_lock(&dbs_mutex);	
	up_threshold = dbs_tuners_ins.up_threshold_awake;
	down_threshold = dbs_tuners_ins.down_threshold_awake;
	up_freq_step = dbs_tuners_ins.up_freq_step_awake;
	down_freq_step = dbs_tuners_ins.down_freq_step_awake;
	is_early_suspend = 0;
	mutex_unlock(&dbs_mutex);
}

static struct early_suspend cpufreq_smartactivev2_early_suspend_info = {
	.suspend = cpufreq_smartactivev2_early_suspend,
	.resume = cpufreq_smartactivev2_late_resume,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
};

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load = 0;
	unsigned int max_load = 0;
	int freq_target;
	struct cpufreq_policy *policy;
	unsigned int j;
	policy = this_dbs_info->cur_policy;

	/* Get Absolute Load */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		if (dbs_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice;
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		if (load > max_load)
			max_load = load;
	}
	if(dbs_tuners_ins.up_freq_step_awake == 0 ||
dbs_tuners_ins.up_freq_step_asleep == 0 || 
dbs_tuners_ins.down_freq_step_awake == 0 || 
dbs_tuners_ins.down_freq_step_asleep == 0)
	return;
	
	/* Check for frequency increase */
	if (max_load > up_threshold) {
		this_dbs_info->down_skip = 0;
		/* if we are already at full speed then break out early */
		if (this_dbs_info->requested_freq == policy->max)
			return;

		freq_target = (up_freq_step * policy->max) / 100;

		/* max freq cannot be less than 100. But who knows.... */
		if (unlikely(freq_target == 0))
			freq_target = 5;

		this_dbs_info->requested_freq += freq_target;
		if (this_dbs_info->requested_freq > policy->max)
			this_dbs_info->requested_freq = policy->max;
		goto setfreq;
	}

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy.
	 */
	if (max_load < down_threshold) {
		freq_target = (down_freq_step * policy->max) / 100;

		this_dbs_info->requested_freq -= freq_target;
		if (this_dbs_info->requested_freq < policy->min)
			this_dbs_info->requested_freq = policy->min;

	/*
	* if we cannot reduce the frequency anymore, break out early
	*/
		if (policy->cur == policy->min)
			return;
		goto setfreq;
	}
setfreq:
	/* set freq */
	__cpufreq_driver_target(policy, this_dbs_info->requested_freq,
				CPUFREQ_RELATION_H);
	return;
}


static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	schedule_delayed_work_on(cpu, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;
		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice)
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}
		this_dbs_info->down_skip = 0;
		this_dbs_info->requested_freq = policy->cur;
		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;
			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			/*
			 * conservative does not implement micro like ondemand
			 * governor, thus we are bound to jiffes/HZ
			 */
			dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		/*
		 * Stop the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTACTIVEV2
static
#endif
struct cpufreq_governor cpufreq_gov_smartactivev2 = {
	.name			= "smartactivev2",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&cpufreq_smartactivev2_early_suspend_info);
#endif
	return cpufreq_register_governor(&cpufreq_gov_smartactivev2);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_smartactivev2);
}


MODULE_AUTHOR("NewWorld <cae11cae@naver.com>");
MODULE_DESCRIPTION("'cpufreq_smartactivev2' - A smartactivev2 governor based on conservative");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTACTIVEV2
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);