Custom Simple Textures For SDF Objects in Gazebo
Created by: Peter Gavriel
Email: pgavriel@mail.middlesex.edu
Date: 11/29/18

BASIS:
For some reason, Gazebo doesn't exactly make it easy to apply simple .jpg/.png
images as textures for objects, so I wanted to find out how to produce custom
textures that I could quickly and easily apply to any objects I made in the future.

I noticed that gazebo has a handful of default textures so I wanted to find out
where those were being defined so that I could make my own in a way that gazebo
would reliably recognize them.

NOTE: There may be a simpler way of accomplishing this using relative paths to material
      files, but gazebo can be quite finicky, so this just describes a method that
      is friendlier to the way gazebo likes to do things.

Turns out they're stored here (by default at least):
/usr/share/gazebo-7/media/materials/scripts/gazebo.material

The materials reference .png textures which are stored in the textures folder
under the materials folder.
Additionally, the materials are defined using OGRE scripts which you can
learn more about here:
https://ogrecave.github.io/ogre/api/latest/manual.html
https://ogrecave.github.io/ogre/api/latest/_material-_scripts.html

PROCESS // SETUP:
1. Grant yourself full permissions of the relevant folders, as they are read only by default:
    sudo chmod 777 /usr/share/gazebo-7/media/materials/*
2. Place whichever *.material files you would like to use under materials/scripts/
3. Place any image files that are being utilized by the *.material file under materials/textures/

USAGE:
After setup is complete, now you can easily reference these textures within your .SDF
model files using two specific tags. Where you place these tags is important, and it
should look something like this (using my caution.material):

<visual name='Visual'>
  <material>
    <script>
      <uri>file://media/materials/scripts/caution.material</uri>
      <name>Caution/Orange4</name>
    </script>
    <ambient>1 1 1 1</ambient>
  </material>
</visual>

The <uri> tag points to the material file you want to use, and the <name> tag
refers specifically to one of the materials defined in that file (You can have
as many materials defined in one file as you want).

Within the specific material you're trying to use, you will see where the image is
referenced. Just make sure the relative path to your .jpg/.png is correct. It should
look like this:

material Caution/Red     //Material <name>
{
  technique
  {
    pass
    {
      ambient 1.0 1.0 1.0 1.0
      texture_unit
      {
        // Relative to the location of the material script
        texture ../textures/redcaution.png
        // Repeat the texture over the surface (4 per face)
        scale 0.5 0.5
      }
    }
  }
}

There should be some examples in this folder to use and aid in understanding.

GLHF
