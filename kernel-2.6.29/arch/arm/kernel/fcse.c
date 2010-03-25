#include <linux/bitops.h>
#include <linux/memory.h>
#include <linux/spinlock.h>
#include <linux/mm.h>

#include <asm/fcse.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#define NR_PIDS (TASK_SIZE / FCSE_PID_TASK_SIZE)
#define PIDS_LONGS ((NR_PIDS + BITS_PER_LONG - 1) / BITS_PER_LONG)

static IPIPE_DEFINE_SPINLOCK(fcse_lock);
static unsigned long fcse_pids_bits[PIDS_LONGS];
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
static unsigned long fcse_pids_cache_dirty[PIDS_LONGS];
static unsigned random_pid;
struct {
	struct mm_struct *last_mm;
	unsigned count;
} per_pid[NR_PIDS];
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */

static void fcse_pid_reference_inner(unsigned pid)
{
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (++per_pid[pid].count == 1)
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
		__set_bit(pid, fcse_pids_bits);
}

static void fcse_pid_dereference(unsigned pid)
{
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (--per_pid[pid].count == 0) {
		__clear_bit(pid, fcse_pids_bits);
		per_pid[pid].last_mm = NULL;
	}
#else /* CONFIG_ARM_FCSE_BEST_EFFORT */
	__clear_bit(pid, fcse_pids_bits);
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
}

int fcse_pid_alloc(void)
{
	unsigned long flags;
	unsigned pid;

	spin_lock_irqsave(&fcse_lock, flags);
	pid = find_next_zero_bit(fcse_pids_bits, NR_PIDS, 1);
	if (pid == NR_PIDS) {
		/* Allocate zero pid last, since zero pid is also used by
		   processes with address space larger than 32MB. */
		if (!test_bit(0, fcse_pids_bits))
			pid = 0;
		else {
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
			if(++random_pid == NR_PIDS)
				random_pid = 0;
			pid = random_pid;
#else /* CONFIG_ARM_FCSE_GUARANTEED */
			spin_unlock_irqrestore(&fcse_lock, flags);
			return -EAGAIN;
#endif /* CONFIG_ARM_FCSE_GUARANTEED */
		}
	}
	fcse_pid_reference_inner(pid);
	spin_unlock_irqrestore(&fcse_lock, flags);

	return pid;
}

void fcse_pid_free(unsigned pid)
{
	unsigned long flags;

	spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid_dereference(pid);
	spin_unlock_irqrestore(&fcse_lock, flags);
}

#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
void fcse_pid_reference(unsigned pid)
{
	unsigned long flags;

	spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid_reference_inner(pid);
	spin_unlock_irqrestore(&fcse_lock, flags);
}

static void fcse_notify_flush_all_inner(struct mm_struct *next)
{
	unsigned long flags;

	spin_lock_irqsave(&fcse_lock, flags);
	switch(ARRAY_SIZE(fcse_pids_cache_dirty)) {
	case 4:
		fcse_pids_cache_dirty[3] = 0UL;
	case 3:
		fcse_pids_cache_dirty[2] = 0UL;
	case 2:
		fcse_pids_cache_dirty[1] = 0UL;
	case 1:
		fcse_pids_cache_dirty[0] = 0UL;
	}
	if (next != &init_mm && next) {
		unsigned pid = next->context.pid >> FCSE_PID_SHIFT;
		__set_bit(pid, fcse_pids_cache_dirty);
	}
	spin_unlock_irqrestore(&fcse_lock, flags);
}

int fcse_needs_flush(struct mm_struct *prev, struct mm_struct *next)
{
	unsigned res, reused_pid = 0, pid = next->context.pid >> FCSE_PID_SHIFT;
	unsigned long flags;

	spin_lock_irqsave(&fcse_lock, flags);
	if (per_pid[pid].last_mm != next) {
		if (per_pid[pid].last_mm)
			reused_pid = test_bit(pid, fcse_pids_cache_dirty);
		per_pid[pid].last_mm = next;
	}
	__set_bit(pid, fcse_pids_cache_dirty);
	spin_unlock_irqrestore(&fcse_lock, flags);

	res = reused_pid
		|| next->context.high_pages
		|| !prev
		|| prev->context.shared_dirty_pages
		|| prev->context.high_pages;

	if (res)
		fcse_notify_flush_all_inner(next);

	return res;
}
EXPORT_SYMBOL_GPL(fcse_needs_flush);

/* Called with mm->mmap_sem write-locked. */
void fcse_relocate_mm_to_null_pid(struct mm_struct *mm)
{
	pgd_t *to = mm->pgd + pgd_index(0);
	pgd_t *from = pgd_offset(mm, 0);
	unsigned len = pgd_index(FCSE_TASK_SIZE) * sizeof(*from);
	unsigned long flags;

	preempt_disable();

	memcpy(to, from, len);
	spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid_dereference(mm->context.pid >> FCSE_PID_SHIFT);
	fcse_pid_reference_inner(0);
	per_pid[0].last_mm = mm;
	spin_unlock_irqrestore(&fcse_lock, flags);

	mm->context.pid = 0;
	fcse_pid_set(0);
	memset(from, '\0', len);
	mb();
	flush_cache_mm(mm);
	flush_tlb_mm(mm);

	preempt_enable();
}

void fcse_notify_flush_all(void)
{
	fcse_notify_flush_all_inner(current->mm);
}
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
