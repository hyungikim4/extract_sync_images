# extract_sync_images

Extract synchronized images based on odometry data

* Create directories
```plain
└── save_path
       ├── event    <-- event camera image
       ├── rgb      <-- rgb image
       └── depth    <-- depth image
```

* Excution
`roslaunch extract_sync_images extract_sync_images.launch`


### Parameters

Launch file available parameters for `extract_sync_images`

|Parameter| Type| Description|
----------|-----|--------
|`init_offset_odom`|*double* |initial offset (m scale). Default `3.`.|
|`interval_odom`|*double*|Save images per interval odometry (m scale). Default `5.`.|
|`save_path`|*String*|Saveing path. Default `/media/khg/HDD1TB/bagfiles/tram_dataset/`.|
