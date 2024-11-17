# 2024 11 17 12:30
## rosout

<font size="1pt">


```markdown
ros@amdc:~/sigyn_ws$ ros2 launch line_finder line_finder.launch.py 
[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2024-11-17-12-25-19-670492-amdc-80177
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [lf-1]: process started with pid [80182]
[lf-1] [INFO] [1731875119.757711337] [LineExtractionROS]: bearing_std_dev: 0.001000
[lf-1] [INFO] [1731875119.757753318] [LineExtractionROS]: frame_id: base_link
[lf-1] [INFO] [1731875119.757757408] [LineExtractionROS]: least_sq_angle_thresh: 0.000100
[lf-1] [INFO] [1731875119.757760978] [LineExtractionROS]: least_sq_radius_thresh: 0.000100
[lf-1] [INFO] [1731875119.757764408] [LineExtractionROS]: max_line_gap: 0.400000
[lf-1] [INFO] [1731875119.757768128] [LineExtractionROS]: max_range: 20.000000
[lf-1] [INFO] [1731875119.757772478] [LineExtractionROS]: min_line_length: 0.500000
[lf-1] [INFO] [1731875119.757775998] [LineExtractionROS]: min_line_points: 9
[lf-1] [INFO] [1731875119.757779209] [LineExtractionROS]: min_range: 0.400000
[lf-1] [INFO] [1731875119.757782579] [LineExtractionROS]: min_split_dist: 0.050000
[lf-1] [INFO] [1731875119.757785909] [LineExtractionROS]: outlier_dist: 0.050000
[lf-1] [INFO] [1731875119.757789379] [LineExtractionROS]: publish_markers: True
[lf-1] [INFO] [1731875119.757792759] [LineExtractionROS]: range_std_dev: 0.020000
[lf-1] [INFO] [1731875119.757796329] [LineExtractionROS]: scan_topic: scan
```

---

## Room corners
| corner | x | y |
|--------|---|---|
| lower-left | -1.44 | -2.43 |
| upper-left | -1.44 | 1.02 |
| upper-right | 1.62 | 1.02 |
| lower-right | 2.35 | -2.43 |
| south-wall-bottom | 1.6 | -0.953 |
| south-wall-top | 1.6 | 0.144 |
| closet-top | 2.33 | -1.47 |
| hallway-bottom | 2.58 | 0.623 |
---

![alt text](<Screenshot from 2024-11-17 12-27-35.png>)

## Lines
| line# | radius | angle | start_x | start_y | end_x | end_y |
|-------|--------|-------|---------|---------|-------|-------|
| 1 | 1.5732 | -3.1411 | -1.5732 | -0.0025 | -1.5720 | -2.2624 |
| 2 | 2.4229 | -1.5708 | -1.5632 | -2.4229 | 1.9161 | -2.4229 |
| 3 | 2.2152 | -0.0020 | 2.2104 | -2.4215 | 2.2119 | -1.6418 |
| 4 | 1.4761 | -0.0021 | 1.4742 | -0.9401 | 1.4762 | 0.0387 |
| 5 | 2.4704 | 0.0051 | 2.4671 | 0.6391 | 2.4630 | 1.4537 |
| 6 | 1.0319 | 1.5710 | 1.4882 | 1.0322 | -1.4170 | 1.0316 |
| 7 | 1.5727 | 3.1415 | -1.5726 | 1.0253 | -1.5727 | 0.0851 |

---
## line segments (w/o covaiances)
ros@amdc:~/sigyn_ws$ ros2 topic echo -f /line_segments 
header:
  stamp:
    sec: 1731875310
    nanosec: 29216355
  frame_id: base_link
line_segments:
- radius: 1.5731842517852783
  angle: -3.141079902648926
  start:
  - -1.5731832124260363
  - -0.002504770104957132
  end:
  - -1.5720241713737517
  - -2.2624262572378218
- radius: 2.422900915145874
  angle: -1.5707900524139404
  start:
  - -1.5632248344342086
  - -2.4229108784365363
  end:
  - 1.9161268102881142
  - -2.42288886099247
- radius: 2.215196371078491
  angle: -0.0020027810242027044
  start:
  - 2.210351039183817
  - -2.4214770184889636
  end:
  - 2.2119124998753223
  - -1.6418318147626523
- radius: 1.4760946035385132
  angle: -0.0020574999507516623
  start:
  - 1.4741635008614258
  - -0.9400960473350228
  end:
  - 1.4761774202059867
  - 0.038721218066253386
- radius: 2.470355272293091
  angle: 0.005100329872220755
  start:
  - 2.467127685829509
  - 0.6390924511928237
  end:
  - 2.462972972085274
  - 1.4536824775297292
- radius: 1.031854510307312
  angle: 1.5710093975067139
  start:
  - 1.4881534123031583
  - 1.0321715942988485
  end:
  - -1.4170065551510962
  - 1.0315526614842032
- radius: 1.572725534439087
  angle: 3.14148211479187
  start:
  - -1.5726122199192905
  - 1.0253445565552337
  end:
  - -1.572716162949457
  - 0.08511851324959645

</font>