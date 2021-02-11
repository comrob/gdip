# Generalized Dubins Interval Problem (GDIP)

[![Build Status](https://travis-ci.com/comrob/gdip.svg?branch=master)](https://travis-ci.com/comrob/gdip)

This repository provides the optimal solution of the GDIP wchich enables to find a tight lower-bound for the Dubins Traveling Salesman Problem with Neighborhoods (DTSPN). The provided source codes are implemented in C++11 and support the following article published at RSS 2018 conference.

```
@INPROCEEDINGS{VANA-RSS-18, 
    AUTHOR    = {Váňa, Petr and Faigl, Jan}, 
    TITLE     = {Optimal Solution of the Generalized Dubins Interval Problem}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.035} 
} 
```

This paper has been nominated for **the best student paper**.

## GDIP example

![GDIP example](https://raw.githubusercontent.com/petvana/images/master/gdip/basic-gdip-example-small.gif)

## DTRP solution

Feasible solution (blue) and the corresponding lower bound (red).

![DTRP solution](https://raw.githubusercontent.com/petvana/images/master/gdip/rss-example-small.gif)

## Basic usage in C++ and library compilation

[C++ library](gdip/GDIP.md)
