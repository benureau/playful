
PREFIX=/opt
DIR=playfulmachines

all: genscripts
	cd Simulations/src && $(MAKE)



install: 
	install -d $(PREFIX)/$(DIR)
	cp -r Experiments resources index.html README LICENSE $(PREFIX)/$(DIR)/
	install -d $(PREFIX)/$(DIR)/Simulations/src
	install --mode 755 `find Simulations -maxdepth 1 -type f` $(PREFIX)/$(DIR)/Simulations/ 
	for F in `find Simulations/src -maxdepth 1 -mindepth 1 -type d`; do \
		install -d  $(PREFIX)/$(DIR)/$$F; done
	for F in `find Simulations/src -maxdepth 2 -type f -name "*" | grep -v "\.depend" |grep -v "/\." | grep -v "\.xml" | grep -v "\.o"`; do cp $$F $(PREFIX)/$(DIR)/$$F; done

uninstall:
	rm -r $(PREFIX)/$(DIR)

genscripts:
	cd Simulations; for F in `find src -mindepth 1 -maxdepth 1 -type d`; do \
	  S=$${F#src/}; cp ../simulationtemplate $$S; chmod +x $$S; done                

distclean: clean

clean:
	cd Simulations/src && $(MAKE) clean
	cd Simulations && rm -f `find -mindepth 1 -maxdepth 1 -type f`

