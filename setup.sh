#!/bin/bash
# Should be run from your ROS workspace directory
pwd
destination="$HOME/.gazebo/models/"
destination2="/usr/share/gazebo-7/media/materials/"
echo
echo "Model Destination: $destination"
echo "Texture Destination: $destination2"
echo
echo "Grant permissions for Gazebo Textures folder:"
sudo chmod 777 /usr/share/gazebo-7/media/materials/*
echo
echo "Copying Models..."
echo "- SDF Pioneer 2 Wheel"; cp -r ./src/uml_3d_race/resources/models/sdf_robots/pioneer_2wd $destination
echo "- SDF Pioneer 4 Wheel"; cp -r ./src/uml_3d_race/resources/models/sdf_robots/pioneer_4wd $destination
echo "- Track Pieces"; cp -r ./src/uml_3d_race/resources/models/track_models/* $destination
echo "- Obstacles"; cp -r ./src/uml_3d_race/resources/models/obstacles/* $destination
echo "Done."
echo
echo "Copying Textures:"
echo "- Caution Material Script"; cp ./src/uml_3d_race/resources/textures/caution_texture/scripts/caution.material $destination2/scripts
echo "- Caution Material Images"; cp ./src/uml_3d_race/resources/textures/caution_texture/textures/* $destination2/textures
echo "- Concrete Material Script"; cp ./src/uml_3d_race/resources/textures/concrete_texture/scripts/concrete.material $destination2/scripts
echo "- Concrete Material Image"; cp ./src/uml_3d_race/resources/textures/concrete_texture/textures/* $destination2/textures
echo "Done."
