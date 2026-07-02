VER = v1-1-1
VERSION = 1.1.1

APP0 = pbsugar
BINDIR = /usr/local/bin
LIBDIR = /usr/local/lib/$(APP0)

APP = $(APP0)-$(VER)
JAR = $(APP).jar
ZIP = $(APP).zip

WEBPAGE = http://bach.istc.kobe-u.ac.jp/pbsugar/
WEBTITLE = PBSugar: SAT-based Pseudo Boolean Constraint Solver
PACKAGE = pbsugar

DOCTITLE = PBSugar version $(VERSION) Core API Specification
JAVADOC  = javadoc -locale en_US \
	-windowtitle '$(DOCTITLE)' -doctitle '$(DOCTITLE)' \
	-header '<em><b>$(DOCTITLE)</b></em><br>' \
	-bottom '<address><a href="$(WEBPAGE)">$(WEBTITLE)</a></address>' \
	-link http://java.sun.com/javase/6/docs/api \
	-exclude org \
	-classpath ../classes:$(JAR1):$(JAR2) -sourcepath ../src -subpackages $(PACKAGE) \
	-public -d ../docs/api

all: javac jar javadoc zip

javac:
	javac -target 1.8 -sourcepath ../src -d ../classes -cp $(JAR1):$(JAR2) \
		../src/pbsugar/*.java \
		../src/pbsugar/*/*.java

jar:
	jar cmf manifest $(JAR) -C ../classes .

javadoc:
	rm -rf ../docs/api
	LC_ALL=C LANG=C $(JAVADOC) $(PACKAGE)

zip:
	rm -f $(ZIP)
	rm -rf $(APP)
	mkdir $(APP)
	cp -pr ../src ../classes ../docs $(APP)
	mkdir $(APP)/bin
	cp -p $(APP0) Makefile manifest $(JAR) $(APP)/bin
	find $(APP) \( -name .svn -o -name CVS -o -name .cvsignore -o -name '*~' \) -exec rm -rf '{}' '+'
	zip -q -r $(ZIP) $(APP)
	rm -rf $(APP)

install:
	mkdir -p $(LIBDIR)
	cp -p $(JAR) $(LIBDIR)
	mkdir -p $(BINDIR)
	cp -p $(APP0) $(APP)
	install $(APP) $(BINDIR)
	cd $(BINDIR); rm -f $(APP0); ln -s $(APP) $(APP0)

clean:
	rm -rf ../docs/api
	rm -rf $(ZIP) $(JAR)
	rm -rf ../classes
	mkdir ../classes

