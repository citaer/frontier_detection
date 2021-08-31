roslaunch sampling_based_frontier_detection  frontier_detection_using_sampling_based.launch
使用效果：
0->224  0.00361124s
224->315 0.00215824s
315->400 0.0023197s
400->493 0.00302388s
493->580 0.0036505s
没有frontier的时候，总共需要0.1ms遍历一遍，对每个轨迹进行分析，需要1*10-6 s
问题分析：主要时间消耗在将frontier聚类的过程当中