#!/bin/bash
cd `dirname $0`/../Simulations
xterm -geom 100x45+810 -e "./guided_twowheeled_cmt -straight 0 -arenasize 30"
