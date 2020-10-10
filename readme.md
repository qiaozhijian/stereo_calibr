#### opencv stereo calibrate
-d 传入标定文件夹
```
./stereo_calib -w=11= -h=8= -s=15= -d=/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/cali= -show=true= 
```
#### matlab stereo calibrate

```
#run matlab stereo calibrate
#save the result
matlabstereo # in matlab
# note: pose of camera left and right should be inv.
./cali_mat
```