
#ifdef CONFIG_SCHED_TUNE

extern int schedtune_normalize_energy(int energy);

#ifdef CONFIG_CGROUP_SCHEDTUNE

extern int schedtune_taskgroup_boost(struct task_struct *tsk);
extern int schedtune_cpu_boost(int cpu);
extern void schedtune_idle(int cpu);

extern void schedtune_enqueue_task(struct task_struct *p, int cpu);
extern void schedtune_dequeue_task(struct task_struct *p, int cpu);

extern int schedtune_accept_deltas(int nrg_delta, int cap_delta,
		struct task_struct *task);

#else /* CONFIG_CGROUP_SCHEDTUNE */

extern int schedtune_accept_deltas(int nrg_delta, int cap_delta);

#define schedtune_enqueue_task(task, cpu) while(0){}
#define schedtune_dequeue_task(task, cpu) while(0){}

#endif /* CONFIG_CGROUP_SCHEDTUNE */

#else /* CONFIG_SCHED_TUNE */

#define schedtune_normalize_energy(energy) energy
#define schedtune_accept_deltas(nrg_delta, cap_delta) nrg_delta

#define schedtune_enqueue_task(task, cpu) while(0){}
#define schedtune_dequeue_task(task, cpu) while(0){}

#endif /* CONFIG_SCHED_TUNE */
