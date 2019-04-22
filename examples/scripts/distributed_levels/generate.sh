#!/usr/bin/env bash

erb type=standalone lv=0 distributed_levels.sdf.erb > standalone.sdf
erb type=primary lv=0 distributed_levels.sdf.erb > primary.sdf
erb type=secondary lv=0 distributed_levels.sdf.erb > secondary.sdf

erb type=standalone lv=1 distributed_levels.sdf.erb > standalone_vis.sdf
erb type=primary lv=1 distributed_levels.sdf.erb > primary_vis.sdf
erb type=secondary lv=1 distributed_levels.sdf.erb > secondary_vis.sdf
