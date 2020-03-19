#!/bin/bash
pwd
destination="$HOME/.gazebo/models/"
destination2="/usr/share/gazebo-7/media/materials/"
echo
echo "Model Destination: $destination"
echo "Texture Destination: $destination2"
echo
echo "Copying Models..."
echo "- Pioneer 2 Wheel"; cp -r ./resources/models/nerpio_2wd $destination
echo "- Pioneer 4 Wheel"; cp -r ./resources/models/nerpio $destination
echo "- Custom ground_plane"; cp -r ./resources/models/ground_plane $destination
echo "- Track Pieces"; cp -r ./resources/models/track_models/* $destination
echo "- Obstacles"; cp -r ./resources/models/obstacles/* $destination
echo "Done."
echo
echo "Grant permissions for Gazebo Textures folder:"
sudo chmod 777 /usr/share/gazebo-7/media/materials/*
echo
echo "Copying Textures:"
echo "- Caution Material Script"; cp ./resources/textures/caution_texture/scripts/caution.material $destination2/scripts
echo "- Caution Material Images"; cp ./resources/textures/caution_texture/textures/* $destination2/textures
echo "- Concrete Material Script"; cp ./resources/textures/concrete_texture/scripts/concrete.material $destination2/scripts
echo "- Concrete Material Image"; cp ./resources/textures/concrete_texture/textures/* $destination2/textures
echo "Done."
