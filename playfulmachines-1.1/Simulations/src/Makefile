#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

include Makefile.conf

all: $(SIMULATIONS)

.PHONY: depend clean $(SIMULATIONS)

$(SIMULATIONS): 
	@echo -e "\n************ Building SIMULATION $@ **************"
	+$(MAKE) -C $@ shared

depend: 
	for dir in $(SIMULATIONS); do \
		$(MAKE) -C $$dir depend; \
	done

clean: 
	for dir in $(SIMULATIONS); do \
		$(MAKE) -C $$dir clean; \
	done



