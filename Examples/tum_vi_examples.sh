#!/bin/bash
pathDatasetTUM_VI='/Datasets/TUM_VI' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching Room 1 with Monocular sensor"
./Monocular/mono_tum_vi ../Vocabulary/ORBvoc.txt Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room1_512.txt dataset-room1_512_mono

echo "Launching Room 2 with Monocular sensor"
./Monocular/mono_tum_vi ../Vocabulary/ORBvoc.txt Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room2_512.txt dataset-room2_512_mono

echo "Launching Room 3 with Monocular sensor"
./Monocular/mono_tum_vi ../Vocabulary/ORBvoc.txt Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room3_512.txt dataset-room3_512_mono

echo "Launching Room 4 with Monocular sensor"
./Monocular/mono_tum_vi ../Vocabulary/ORBvoc.txt Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_mono

echo "Launching Room 5 with Monocular sensor"
./Monocular/mono_tum_vi ../Vocabulary/ORBvoc.txt Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room5_512.txt dataset-room5_512_mono

echo "Launching Room 6 with Monocular sensor"
./Monocular/mono_tum_vi ../Vocabulary/ORBvoc.txt Monocular/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room6_512.txt dataset-room6_512_mono


#------------------------------------
# Stereo Examples
echo "Launching Room 1 with Stereo sensor"
./Stereo/stereo_tum_vi ../Vocabulary/ORBvoc.txt Stereo/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Stereo/TUM_TimeStamps/dataset-room1_512.txt dataset-room1_512_stereo

echo "Launching Room 2 with Stereo sensor"
./Stereo/stereo_tum_vi ../Vocabulary/ORBvoc.txt Stereo/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam1/data Stereo/TUM_TimeStamps/dataset-room2_512.txt dataset-room2_512_stereo

echo "Launching Room 3 with Stereo sensor"
./Stereo/stereo_tum_vi ../Vocabulary/ORBvoc.txt Stereo/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam1/data Stereo/TUM_TimeStamps/dataset-room3_512.txt dataset-room3_512_stereo

echo "Launching Room 4 with Stereo sensor"
./Stereo/stereo_tum_vi ../Vocabulary/ORBvoc.txt Stereo/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Stereo/TUM_TimeStamps/dataset-room4_512.txt dataset-room4_512_stereo

echo "Launching Room 5 with Stereo sensor"
./Stereo/stereo_tum_vi ../Vocabulary/ORBvoc.txt Stereo/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam1/data Stereo/TUM_TimeStamps/dataset-room5_512.txt dataset-room5_512_stereo

echo "Launching Room 6 with Stereo sensor"
./Stereo/stereo_tum_vi ../Vocabulary/ORBvoc.txt Stereo/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam1/data Stereo/TUM_TimeStamps/dataset-room6_512.txt dataset-room6_512_stereo


#------------------------------------
# Monocular-Inertial Examples
echo "Launching Corridor 1 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor1_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-corridor1_512.txt Monocular-Inertial/TUM_IMU/dataset-corridor1_512.txt dataset-corridor1_512_monoi

echo "Launching Corridor 2 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor2_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-corridor2_512.txt Monocular-Inertial/TUM_IMU/dataset-corridor2_512.txt dataset-corridor2_512_monoi

echo "Launching Corridor 3 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor3_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-corridor3_512.txt Monocular-Inertial/TUM_IMU/dataset-corridor3_512.txt dataset-corridor3_512_monoi

echo "Launching Corridor 4 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt Monocular-Inertial/TUM_IMU/dataset-corridor4_512.txt dataset-corridor4_512_monoi

echo "Launching Corridor 5 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor5_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-corridor5_512.txt Monocular-Inertial/TUM_IMU/dataset-corridor5_512.txt dataset-corridor5_512_monoi


echo "Launching Magistrale 1 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Monocular-Inertial/TUM_IMU/dataset-magistrale1_512.txt dataset-magistrale1_512_monoi

echo "Launching Magistrale 2 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale2_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-magistrale2_512.txt Monocular-Inertial/TUM_IMU/dataset-magistrale2_512.txt dataset-magistrale2_512

echo "Launching Magistrale 3 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale3_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-magistrale3_512.txt Monocular-Inertial/TUM_IMU/dataset-magistrale3_512.txt dataset-magistrale3_512_monoi

echo "Launching Magistrale 4 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale4_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-magistrale4_512.txt Monocular-Inertial/TUM_IMU/dataset-magistrale4_512.txt dataset-magistrale4_512_monoi

echo "Launching Magistrale 5 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale5_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-magistrale5_512.txt Monocular-Inertial/TUM_IMU/dataset-magistrale5_512.txt dataset-magistrale5_512_monoi

echo "Launching Magistrale 6 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale6_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-magistrale6_512.txt Monocular-Inertial/TUM_IMU/dataset-magistrale6_512.txt dataset-magistrale6_512_monoi


echo "Launching Outdoor 1 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors1_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors1_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors1_512.txt dataset-outdoors1_512_monoi

echo "Launching Outdoor 2 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors2_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors2_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors2_512.txt dataset-outdoors2_512_monoi

echo "Launching Outdoor 3 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors3_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors3_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors3_512.txt dataset-outdoors3_512_monoi

echo "Launching Outdoor 4 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors4_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors4_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors4_512.txt dataset-outdoors4_512_monoi

echo "Launching Outdoor 5 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors5_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors5_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors5_512.txt dataset-outdoors5_512_monoi

echo "Launching Outdoor 6 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors6_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors6_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors6_512.txt dataset-outdoors6_512_monoi

echo "Launching Outdoor 7 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors7_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors7_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors7_512.txt dataset-outdoors7_512_monoi

echo "Launching Outdoor 8 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512_far.yaml "$pathDatasetTUM_VI"/dataset-outdoors8_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-outdoors8_512.txt Monocular-Inertial/TUM_IMU/dataset-outdoors8_512.txt dataset-outdoors8_512_monoi


echo "Launching Room 1 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-room1_512.txt Monocular-Inertial/TUM_IMU/dataset-room1_512.txt dataset-room1_512_monoi

echo "Launching Room 2 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-room2_512.txt Monocular-Inertial/TUM_IMU/dataset-room2_512.txt dataset-room2_512_monoi

echo "Launching Room 3 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-room3_512.txt Monocular-Inertial/TUM_IMU/dataset-room3_512.txt dataset-room3_512_monoi

echo "Launching Room 4 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-room4_512.txt Monocular-Inertial/TUM_IMU/dataset-room4_512.txt dataset-room4_512_monoi

echo "Launching Room 5 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-room5_512.txt Monocular-Inertial/TUM_IMU/dataset-room5_512.txt dataset-room5_512_monoi

echo "Launching Room 6 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-room6_512.txt Monocular-Inertial/TUM_IMU/dataset-room6_512.txt dataset-room6_512_monoi


echo "Launching Slides 1 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-slides1_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-slides1_512.txt Monocular-Inertial/TUM_IMU/dataset-slides1_512.txt dataset-slides1_512_monoi

echo "Launching Slides 2 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-slides2_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-slides2_512.txt Monocular-Inertial/TUM_IMU/dataset-slides2_512.txt dataset-slides2_512_monoi

echo "Launching Slides 3 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-slides3_512_16/mav0/cam0/data Monocular-Inertial/TUM_TimeStamps/dataset-slides3_512.txt Monocular-Inertial/TUM_IMU/dataset-slides3_512.txt dataset-slides3_512_monoi


# MultiSession Monocular Examples
echo "Launching Room 1, Magistrale 1, Magistrale 5 and Slides 1 in the same session with Stereo-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt "$pathDatasetTUM_VI"/dataset-magistrale5_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale5_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale5_512.txt "$pathDatasetTUM_VI"/dataset-slides1_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-slides1_512.txt Stereo-Inertial/TUM_IMU/dataset-slides1_512.txt dataset-room1_mag1_mag5_slides1_monoi

echo "Launching all Rooms (1-6) in the same session with Stereo-Inertial sensor"
./Monocular-Inertial/mono_inertial_tum_vi ../Vocabulary/ORBvoc.txt Monocular-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Stereo-Inertial/TUM_IMU/dataset-room2_512.txt "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Stereo-Inertial/TUM_IMU/dataset-room3_512.txt "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Stereo-Inertial/TUM_IMU/dataset-room4_512.txt "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Stereo-Inertial/TUM_IMU/dataset-room5_512.txt "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_monoi

#------------------------------------
# Stereo-Inertial Examples
echo "Launching Corridor 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-corridor1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-corridor1_512.txt Stereo-Inertial/TUM_IMU/dataset-corridor1_512.txt dataset-corridor1_512_stereoi

echo "Launching Corridor 2 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-corridor2_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-corridor2_512.txt Stereo-Inertial/TUM_IMU/dataset-corridor2_512.txt dataset-corridor2_512_stereoi

echo "Launching Corridor 3 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-corridor3_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-corridor3_512.txt Stereo-Inertial/TUM_IMU/dataset-corridor3_512.txt dataset-corridor3_512_stereoi

echo "Launching Corridor 4 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-corridor4_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt Stereo-Inertial/TUM_IMU/dataset-corridor4_512.txt dataset-corridor4_512_stereoi

echo "Launching Corridor 5 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-corridor5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-corridor5_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-corridor5_512.txt Stereo-Inertial/TUM_IMU/dataset-corridor5_512.txt dataset-corridor5_512_stereoi


echo "Launching Magistrale 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt dataset-magistrale1_512_stereoi

echo "Launching Magistrale 2 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale2_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale2_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale2_512.txt dataset-magistrale2_512_stereoi

echo "Launching Magistrale 3 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale3_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale3_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale3_512.txt dataset-magistrale3_512_stereoi

echo "Launching Magistrale 4 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale4_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale4_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale4_512.txt dataset-magistrale4_512_stereoi

echo "Launching Magistrale 5 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale5_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale5_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale5_512.txt dataset-magistrale5_512_stereoi

echo "Launching Magistrale 6 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-magistrale6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale6_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale6_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale6_512.txt dataset-magistrale6_512_stereoi


echo "Launching Outdoor 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors1_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors1_512.txt outdoors1_512_stereoi

echo "Launching Outdoor 2 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors2_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors2_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors2_512.txt outdoors2_512_stereoi

echo "Launching Outdoor 3 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors3_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors3_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors3_512.txt outdoors3_512

echo "Launching Outdoor 4 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors4_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors4_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors4_512.txt outdoors4_512

echo "Launching Outdoor 5 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors5_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors5_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors5_512.txt outdoors5_512_stereoi

echo "Launching Outdoor 6 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors6_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors6_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors6_512.txt outdoors6_512_stereoi

echo "Launching Outdoor 7 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors7_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors7_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors7_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors7_512.txt outdoors7_512_stereoi

echo "Launching Outdoor 8 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512_outdoors.yaml "$pathDatasetTUM_VI"/dataset-outdoors8_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-outdoors8_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-outdoors8_512.txt Stereo-Inertial/TUM_IMU/dataset-outdoors8_512.txt outdoors8_512_stereoi


echo "Launching Room 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Stereo-Inertial/TUM_IMU/dataset-room1_512.txt dataset-room1_512_stereoi

echo "Launching Room 2 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Stereo-Inertial/TUM_IMU/dataset-room2_512.txt dataset-room2_512_stereoi

echo "Launching Room 3 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Stereo-Inertial/TUM_IMU/dataset-room3_512.txt dataset-room3_512_stereoi

echo "Launching Room 4 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Stereo-Inertial/TUM_IMU/dataset-room4_512.txt dataset-room4_512_stereoi

echo "Launching Room 5 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Stereo-Inertial/TUM_IMU/dataset-room5_512.txt dataset-room5_512_stereoi

echo "Launching Room 6 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-room6_512_stereoi


echo "Launching Slides 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-slides1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-slides1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-slides1_512.txt Stereo-Inertial/TUM_IMU/dataset-slides1_512.txt dataset-slides1_512_stereoi

echo "Launching Slides 2 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-slides2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-slides2_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-slides2_512.txt Stereo-Inertial/TUM_IMU/dataset-slides2_512.txt dataset-slides2_512_stereoi

echo "Launching Slides 3 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-slides3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-slides3_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-slides3_512.txt Stereo-Inertial/TUM_IMU/dataset-slides3_512.txt dataset-slides3_512_stereoi


# MultiSession Stereo-Inertial Examples
echo "Launching Room 1, Magistrale 1, Magistrale 5 and Slides 1 in the same session with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt "$pathDatasetTUM_VI"/dataset-magistrale5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-magistrale5_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-magistrale5_512.txt Stereo-Inertial/TUM_IMU/dataset-magistrale5_512.txt "$pathDatasetTUM_VI"/dataset-slides1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-slides1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-slides1_512.txt Stereo-Inertial/TUM_IMU/dataset-slides1_512.txt dataset-room1_mag1_mag5_slides1_stereoi

echo "Launching all Rooms (1-6) in the same session with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room1_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt Stereo-Inertial/TUM_IMU/dataset-room1_512.txt "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room2_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Stereo-Inertial/TUM_IMU/dataset-room2_512.txt "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room3_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Stereo-Inertial/TUM_IMU/dataset-room3_512.txt "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room4_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Stereo-Inertial/TUM_IMU/dataset-room4_512.txt "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room5_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Stereo-Inertial/TUM_IMU/dataset-room5_512.txt "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam0/data "$pathDatasetTUM_VI"/dataset-room6_512_16/mav0/cam1/data Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_stereoi
