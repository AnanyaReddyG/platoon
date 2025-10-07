This file contains instructions to understand and debug the setup.     


## Transforms running in the background    

- To understand the transforms that are being used in the background, run :  
  
  ```
  ros2 run tf2_tools view_frames
  ```    
  It will read the topics data from `/tf` and `/tf_static` and save the transforms in a pdf file within the current directory.     

  The content of that file will be like this    

    <div align="center">
    <img src="media/ns_burger_bringup/ns_bringup_tf_frames.png" alt="Transform data for burger2" />
    </div> 

  - Now you can understand that we've only namespaced all the frames, nodes, topics that are related to a specific robot.    

  - This will be used to differentiate robots and their data from other robots within the same ROS network.    

  - The only frame that is common is the ***world*** which is basically the frame wrt. whiich the odometry of all the robots is getting initialized.      

  - The ***/tf*** & ***/tf_static*** topics contains the global transform data of all the namespaced robots wrt. the ***world*** frame.      

  - This is becase we need a global transform data which can keep track of transform of all the robots.   

  - In the ***updated_turtlebot3_node*** package, the code to initialize odometry of a robot to a custom value has been implemented and the frame_id & custom transform has been utilized from [***params***](/robot_bringup/param/) folder.


## Flow of information     

- Understand flow of information using rqt_graph :     

  To understand what i said in my earlier discussion and visualize how the information is getting traversed between the nodes & topics within the ROS environment, we can use rqt_graph.    

- Run this command to open rqt_graph : 

  ```bash
  ros2 run rqt_graph rqt_graph
  ```    
  Refresh the rqt_window and select specific checkpoints and it will look something like this    

    <div align="center">
    <img src="media/ns_burger_bringup/ns_bringup_rqt.png" alt="Transform data for burger2" />
    </div>   
    
  It is clear that all the nodes and topics are namespaced except the ***/tf*** and ***/tf_static*** topics and i already gave the reason for that.   
  


## Side Note :    

- The reason why i included this so that one can really understand what am i trying to do and how to visualize those changes.     
 
- Now you need to repeat this process for all the robot's you're going to run simulataneously.     

- To publish any data like velocity or subscribe any data like imu, odom or scan make sure you are using the namespaced topic name of that specific robot.