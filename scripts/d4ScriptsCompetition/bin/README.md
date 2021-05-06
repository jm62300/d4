# How to use

In order to run the program in model counting mode you only need to use the following command line:

``` bash
$ ./execute.sh -m test.cnf
s mc 7
```

The commands lines for the weighted model counting and projected model counting are quite similar:

``` bash
$ ./execute.sh -w test.wcnf
s wmc 23.00000

$ ./execute.sh -p test.pcnf
s pmc 3
```

The result is written using the competition style, meaning:
* comments start with the character **c**
* the number of models computed follows the character **s**
  * s mc VALUE: in the case we want to compute the number of models
  * s wmc VALUE: in the case we want to compute the weighted number of models
  * s pmc VALUE: in the case we want to compute the projected number of models
