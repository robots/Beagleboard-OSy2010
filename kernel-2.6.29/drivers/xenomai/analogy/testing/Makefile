ifeq ($(PATCHLEVEL),6)

# Makefile frag for Linux v2.6

EXTRA_CFLAGS += -D__IN_XENOMAI__ -Iinclude/xenomai

obj-$(CONFIG_XENO_DRIVERS_ANALOGY_FAKE) += analogy_fake.o
obj-$(CONFIG_XENO_DRIVERS_ANALOGY_LOOP) += analogy_loop.o

analogy_fake-y := fake.o

analogy_loop-y := loop.o

else

# Makefile frag for Linux v2.4

O_TARGET := built-in.o

obj-$(CONFIG_XENO_DRIVERS_ANALOGY_FAKE) += analogy_fake.o
obj-$(CONFIG_XENO_DRIVERS_ANALOGY_LOOP) += analogy_loop.o

analogy_fake-objs := fake.o

analogy_loop-objs := loop.o

export-objs := $(analogy_fake-objs) $(analogy_loop-objs)

EXTRA_CFLAGS += -D__IN_XENOMAI__ -I$(TOPDIR)/include/xenomai -I$(TOPDIR)/include/xenomai/compat

include $(TOPDIR)/Rules.make

analogy_fake.o: $(analogy_fake-objs)
	$(LD) -r -o $@ $(analogy_fake-objs)

analogy_loop.o: $(analogy_loop-objs)
	$(LD) -r -o $@ $(analogy_loop-objs)

endif
