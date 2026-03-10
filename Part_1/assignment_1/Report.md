Report for dataset 2

## Issues Encountered in Dataset 2

The second dataset presents multiple **false positives** due to the fact that portions of buildings and trees are erroneously grouped into cluster. This occurs because the Euclidean clustering algorithm relies exclusively on spatial distance and number of points, without being able to distinguishing vehicles from noise.

The implemented solutions (PCL built-in or simple Euclidean clustering) are insufficient, and more advanced approaches are required, such as feature-based clustering.

Furthermore, small vehicles (motorcycles, bicycles) are classified as noise and are not clustered, representing **false negatives** in the system. This is due to the size and also the distance of them from the ego vehicle causing insufficient number of points in the cluster.