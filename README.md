# Simulation-of-interactions-between-pedestrians-and-vehicles-at-midblock-crossings

碩士論文 在中間塊行人穿越道的行人與車輛互動的模擬<br>

![4](https://user-images.githubusercontent.com/49235533/210380820-69aaf60e-9dd0-4dc7-9800-bcb2aebb8d5b.png)

## Install & Build Carla
請先安裝Carla Windows Unreal Editor版本0.9.12，並且跟著官網的指示安裝需要的軟體，並建置完成。<br>
● Carla官網網頁:	https://carla.readthedocs.io/en/latest/start_quickstart/#carla-versions-prior-to-0912
    
## Run Code  
 將ThesisCode.zip內的資料夾解壓縮到路徑 carla/PythonAPI底下。
 ![image](https://user-images.githubusercontent.com/49235533/210377368-adf9632e-7ce8-4a69-b015-0c2c2b0d3c2c.png)
 
 開啟CarlaUE4
 ![image](https://user-images.githubusercontent.com/49235533/210377757-383e3cf7-82be-4d9e-8883-337f0ed9560c.png)
 
 開好後到Content Browser之中，點選資料夾移動檔案路徑到Content/Carla/Maps
 ![image](https://user-images.githubusercontent.com/49235533/210377794-f766c39b-73b5-4e42-abb7-893aa232c762.png)
 
 選擇Town10HD
 ![image](https://user-images.githubusercontent.com/49235533/210377825-3f8aea8e-116c-4653-a896-c945e67a0916.png)
 
 開好後會像這樣
 ![image](https://user-images.githubusercontent.com/49235533/210377854-63e9fc17-79e4-45a4-a4ab-2f32fc149048.png)
 
 可透過wasd和滑鼠移動視角
 ![image](https://user-images.githubusercontent.com/49235533/210377884-7fe059b8-c7c0-48f3-a49e-7b73904618d5.png)
 
 開啟Carla Server , 點選畫面右上play旁邊的小箭頭，按Standalone game
 ![image](https://user-images.githubusercontent.com/49235533/210377918-b32940a6-bf77-47b6-9d59-617b45c77074.png)
 
 開啟後會像這樣，可透過wasd和滑鼠移動視角
 ![image](https://user-images.githubusercontent.com/49235533/210377963-39720e42-938d-47d5-8985-0e3c9c233ef9.png)
 
 ![image](https://user-images.githubusercontent.com/49235533/210378004-8b401d65-f05e-4975-9407-06feb17aa557.png)
 
 回到資料夾Thesis Code裡面，開啟scenario.csv 
 ![image](https://user-images.githubusercontent.com/49235533/210378088-2d9eaaaf-ac17-4ffa-9a90-1564954888da.png)
 
 開啟後會看到兩列字串，第一列代表參數名稱，第二列是參數，這參數決定了等下會跑論文中的哪一種情景，共四種
 ![image](https://user-images.githubusercontent.com/49235533/210378126-805eedc8-a641-42ff-bfe7-72df578531a9.png)

 可更改第二列參數，有四種可以更改
 
    scenario_d  > First scenario
    scenario_e  > Lane Changing scenario
    scenario_f  > Evacuating scenario
    scenario_g  > User Interactive scenario

 此README以 scenario_f 為例子，回到carla/PythonAPI/ThesisCode，開啟para_config資料夾
 ![image](https://user-images.githubusercontent.com/49235533/210378230-37be4553-a2b0-4283-984a-d7acb97d84b7.png)
 
 點選scenario_f
 ![image](https://user-images.githubusercontent.com/49235533/210378311-4240279a-bca2-499b-bc1b-d57692346275.png)

開啟parameter.csv
![image](https://user-images.githubusercontent.com/49235533/210378261-4441c294-c663-4b33-ab6a-12ae426ae037.png)

開啟後會像這樣，共兩列，第一列是參數名稱，第二列是參數
![image](https://user-images.githubusercontent.com/49235533/210378375-c25bd97e-5e55-4a31-ba88-0667a2a55f68.png)

共有11個參數:

    veh_arrival_rate:每幾frame在spawn point隨機出現一台車
    total_frame:這次模擬所要花的總frame數量
    min_ped_speed: 行人最小移動速度
    max_ped_speed:行人最大移動速度
    min_veh_desire_speed:車最小desire speed
    max_veh_desire_speed:車最大desire speed
    w1:Degree of passage參數
    w2:Region distance參數
    w3:Decay rate參數
    w4:Region flow ratio參數
    w5:Occupancy ratio參數
    
設定好參數之後，開啟x64 Native Tools Command Prompt for VS 2019(以下簡稱x64 cmd prompt)，cd將路徑移動到ThesisCode內。
![image](https://user-images.githubusercontent.com/49235533/210378439-7dd6e546-9060-4ed0-9280-29794950a05c.png)

在carla server是開啟的情況下(務必開啟carla server)，輸入python update.py，他會根據剛剛設定的參數去跑模擬
![image](https://user-images.githubusercontent.com/49235533/210378460-b9b5ca5c-2a13-442a-a1c0-e0d7c9bbb4eb.png)

    python update.py

執行python update前的carla server(以scenario_f為例)
![image](https://user-images.githubusercontent.com/49235533/210378489-fde84eb6-47cc-42df-ac68-4e76806289fb.png)

執行python update後的carla server，開始跑模擬
![image](https://user-images.githubusercontent.com/49235533/210378517-e4ab9ec5-6e52-4e2c-8bc6-1b9ac703bf6a.png)

此時等他跑完，他會根據剛剛total frame設多少跑多少frame。

## 動畫的回放和將動畫存成多張圖片

每一次模擬跑完後，會將該次模擬的記錄檔存在對應的資料夾，跑完模擬系統會將檔案存在資料夾，資料夾檔名會以流水號往下命名。如我剛剛跑完了一次scenario_f的模擬，跑完後carla/PythonAPI/ThesisCode/scenario_f內會出現該次模擬的記錄檔。記錄檔為一個csv檔案，會存在一個資料夾內，資料夾名稱會以流水號來命名。
![image](https://user-images.githubusercontent.com/49235533/210378604-18415e1a-9327-47ba-8425-68555a1dc264.png)

此時進入到路徑PythonAPI/ThesisCode/save_image_para
![image](https://user-images.githubusercontent.com/49235533/210378639-e59576f1-f3e7-418c-aab5-5495e055c133.png)

開啟parameter.csv
![image](https://user-images.githubusercontent.com/49235533/210378668-a9bd70b0-6d86-4f9f-a1a3-95616c3790cb.png)

有兩個參數

    no_render_mode
    save_simulate_image

no_render_mode:是否開啟no_render_mode，為True時則Carla再回放動畫時不會顯示在Carla Server上面(Carla Server畫面全黑，一班用於save_simulate_image為True時)。    
save_simulate_image:是否將動畫存成多張圖片檔，為True會將動畫以每一張圖片的形式存入D://scenario_…內。
設定好參數後，開啟Carla server(開啟方法在RunCode scetion之中，務必開啟carla server)，並在x64 cmd prompt cd到carla/PythonAPI/ThesisCode中，輸入執行:

    python scenario_anim.py
    
![image](https://user-images.githubusercontent.com/49235533/210379168-6c4df980-6cb8-42ef-8830-2c0ee2be588d.png)

## 動畫的製作
請先安裝ffmpeg，安裝後請設定好環境變數<br>
● 官網網頁 : http://ffmpeg.org/ <br>

進入動畫每張圖所儲存的路徑，開啟cmd，或是開啟cmd後cd進入動畫每張圖所儲存的路徑。
![image](https://user-images.githubusercontent.com/49235533/210379323-6b7e6e02-6ac5-4ade-aac9-938bb0aef730.png)

輸入並且執行 : 

    ffmpeg -framerate 你要的fps -start_number 要從哪張開始(整數) -i %d.png -c:v libx264 -vf  scale='w=1280:h=720'  -pix_fmt yuv420p 檔名.mp4
    
![image](https://user-images.githubusercontent.com/49235533/210379483-4b7307cb-76c9-45b4-81ee-ceaa3e5b3bed.png)

跑完後，往下拉可以看到生成的影片。(下範例圖的檔名與上圖所輸入不同)

![image](https://user-images.githubusercontent.com/49235533/210379569-f7da3bab-0fe4-4133-b997-6826b22dedbe.png)


## 碩士論文連結:
https://drive.google.com/file/d/18I9jf5T8fXtxfdAsIO1wi2Ss038o23e2/view?usp=drivesdk

## Demo Videos:
1. 應用場景 time step = (1/30) second, FPS = 30:
    1. 塞車 https://www.youtube.com/watch?v=LZaUY5JHa-k
    2. 逃生 https://www.youtube.com/watch?v=ao1J6ZqDLDU
    3. 違停 https://www.youtube.com/watch?v=iewIjPVY5-o
    4. 彎道 https://www.youtube.com/watch?v=fogdDFNxiAU
    
2. 不同權重組合下行人的移動行為 time step = (1/10) second, FPS = 10:
    1. 行人偏向往較寬敞的車縫移動 https://www.youtube.com/watch?v=AhqmtyIU5vo
    2. 行人偏向往離他較近的車縫移動 https://www.youtube.com/watch?v=7suVBE_vCDI
    3. 行人偏向往正在成長的車縫移動 https://www.youtube.com/watch?v=8fE66Sr2F_U
    4. 行人偏向往行人流向較順暢的車縫移動 https://www.youtube.com/watch?v=MMjtTvvWKVA
    5. 行人偏向往其他行人較少(較空)的車縫移動 https://www.youtube.com/watch?v=kBIRI2C_jTk
    
## 口試簡報:
https://docs.google.com/presentation/d/19LrpqygApTEtPSxt8VePFuzmMxURK7u5/edit?usp=share_link&ouid=104081438213918509325&rtpof=true&sd=true
