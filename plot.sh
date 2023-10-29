#!/bin/sh
evo_res results/ape/euroc_orb.zip results/ape/euroc_akaze.zip --save_table results/ape/euroc_table.csv --save_plot results/ape/euroc_plot.pdf
evo_res results/ape/tum_orb.zip results/ape/tum_akaze.zip --save_table results/ape/tum_table.csv --save_plot results/ape/tum_plot.pdf
evo_res results/ape/euroc_orb.zip results/ape/euroc_akaze.zip --save_table results/ape/euroc_table.csv --save_plot results/ape/euroc_plot.pdf
evo_res results/rpe/euroc_orb.zip results/rpe/euroc_akaze.zip --save_table results/rpe/euroc_table.csv --save_plot results/rpe/euroc_plot.pdf
evo_res results/rpe/tum_orb.zip results/rpe/tum_akaze.zip --save_table results/rpe/tum_table.csv --save_plot results/rpe/tum_plot.pdf
evo_res results/rpe/euroc_orb.zip results/rpe/euroc_akaze.zip --save_table results/rpe/euroc_table.csv --save_plot results/rpe/euroc_plot.pdf
