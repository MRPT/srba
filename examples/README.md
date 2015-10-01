Examples based on the datasets in `datasets` directory.

`run_demo_tutorials_monoSLAM`

```
srba-slam --se3 --lm-3d --obs MonocularCamera \
  -d OUT_dataset_tutorials_mono_SENSOR.txt \
  --sensor-params-cfg-file OUT_dataset_tutorials_mono_CAMCALIB.txt \
  --submap-size 10 --max-spanning-tree-depth 3 --max-optimize-depth 3 \
  --verbose 1 --noise 0.5 --add-noise # --step-by-step
```


`run_demo_world-2d-30k-rel-graph-slam`

```
srba-slam --se2 --graph-slam -d dataset_30k_rel_graph_slam_SENSOR.txt \
  --submap-size 10 --max-spanning-tree-depth 3 --max-optimize-depth 3 \
  --verbose 1 --noise 0.001 --noise-ang 0.2 --add-noise \
  --gt-map dataset_30k_rel_graph_slam_GT_MAP.txt \
  --gt-path dataset_30k_rel_graph_slam_GT_PATH.txt # --step-by-step
```


