set(QueuePathPlannerSRC
    queue_path_planner.cpp
    QueuePathPlanner_append.cpp
    QueuePathPlanner_core.cpp
    QueuePathPlanner_feedback.cpp
    QueuePathPlanner_remove.cpp
    QueuePathPlanner_task.cpp
    QueuePathPlanner_planning.cpp
)

set(QueueTrajectoryControllerSRC
    queue_trajectory_controller.cpp
)

add_executable(queue_path_planner ${QueuePathPlannerSRC})
add_dependencies(queue_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(queue_path_planner pathplanning travanalyzerpcl pathplanningutils ${catkin_LIBRARIES}  ${PCL_LIBS_DEPS})

# this is a dummy node, to simulate the controller handshake
add_executable(queue_trajectory_controller ${QueueTrajectoryControllerSRC})
add_dependencies(queue_trajectory_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(queue_trajectory_controller  ${catkin_LIBRARIES})
