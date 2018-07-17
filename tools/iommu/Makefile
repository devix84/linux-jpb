include ../scripts/Makefile.include

all: smmute

smmute: FORCE
	$(call descend,$@)

smmute-clean: FORCE
	$(call descend,smmute/,clean)

clean: smmute-clean FORCE

FORCE:

.PHONY: FORCE
