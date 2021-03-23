all:
	make -C cnfuzzdd2009
	cp cnfuzzdd2009/cnfuzz .
	make -C minisatSolver/core
	cp minisatSolver/core/minisat .

clean:
	make clean -C cnfuzzdd2009 
	make clean -C minisatSolver/core
	rm -f cnfuzz
	rm -f minisat
	rm -f *~
