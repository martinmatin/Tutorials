# 1. Getting in touch with Fusion360
Fusion360 is a 3D modeling software used by many professionals and hobbyists to create pieces. You can directly send your models to slicers in order to print them or export the 2D plans to build it your own. The following few chapters should give you a global overview of how Fusion360 works and what you can use it for. We'll try to orient our writing to learn your to design in order to print your models later and will end with some tips about the printing of your designs.

To complete your learning we strongly recommend you subscribe to <a href="https://www.youtube.com/channel/UCo29kn3d9ziFUZGZ50VKvWA">Lars Christensen's Youtube channel</a>. He's a master in the use of Fusion360 and has very well made videos about every problem of question you might have.

3 videos in particular should hold your attention to get started:
<a href="https://www.youtube.com/watch?v=A5bc9c3S12g"> Fusion 360 Tutorial for Absolute Beginners— Part 1</a> and of course <a href="https://www.youtube.com/watch?v=HXRMzJWo0-Q"> Fusion 360 Tutorial for Absolute Beginners— Part 2</a>  and <a href="https://www.youtube.com/watch?v=zS8dYA_Iluc&t=614s"> Fusion 360 Tutorial for Absolute Beginners— Part 3</a>.  

## Fusion360 environment
The first window you get when opening Fusion360 for the first time should be this.


 By clicking on ![img](../img/mechanical/square.png) you access to the repository holding all your folders.
 A good practice is to sort them into other repositories according to their relevance.
 ![FOLDERS](../img/mechanical/FOLDERS.png)

## Project view

![img](../img/mechanical/Fusion1.png)

We'll let you listen to M.Christensen by clicking on the links of the videos we gave earlier to learn the basics of Fusion360. He's an expert on the subject and honestly the only good way to learn the kind of software is by watching someone do and copy. Please don't learn the shortcuts by heart, you'll learn the most used ones by practicing.

# 2. Main Functions and Shotcuts
You should already be able to do your first model now but in case of you misted some of the multiple things explained by "Lars", we would like to give you a quick refresh.

To begin with, note that all function are easily available trough the "s" key shortcut. Just type some word related on what you are searching for and we are pretty sure that you'll find it fast. *Note that you can save them by clicking on the curved arrow.*

## 2.1. Shortcuts
You don't have to study them but with your first hour of practice you should already know them. If it's still not the case, you'll find them here:
- S=Model Toolbox


- L=Line
- C=Circle
- X=Construction
- D=Dimension


- Q=Push/Pull
- M=Move
- J=Joint
- T=Trim (delete some lines)


- center btn = Pan (move the view)
- Maj+ center btn  = 3D move

## 2.2. Useful functions

### Offset
Offset is a very handy tool when it comes to draw parallel lines.
![img](../img/mechanical/Offset1.png)

The tools works only in sketch mode and you have to already have a reference line (or a shape). Type the "o" key to open the "offset menu", you then have to select the reference line you want. You can either drag and drop the cursor with your mouse or type the value you wish the offset to be.
![img](../img/mechanical/Offset_drag.png)![caca](../img/mechanical/OFFSET_VALUE.png)
You'll end with the line/shape you selected offset as you wanted.
![img](../img/mechanical/Offset_finish.png)

### Sweep
Another useful tool to make pipe, slide and so one is the "sweep". Start by doing a sketch on one face. for us it will be two circle.
![img](../img/mechanical/Sweep_1.png)

Then stop your current sketch. Go to another face. Start a new sketch ("s" and then "sketch"). Use "spine" ("s" and then "spine") and draw a line following as much point as you need.
![img](../img/mechanical/Sweep_2.png)

You'll be able now to use "Sweep", select the space between the two circle as "profile", select the line as "path" and you'll automatically make a beautiful pipe!
![img](../img/mechanical/Sweep_3.png)


### Revolve
Revolve allows you to create object as if you turned them on a lathe. It comes in handy to make wheels, cannons, ...
To used it you have to first create a shape around an axis. When you create that shape, you have to imagine it round. So you only have to draw "a half cut in your future object".
![img](../img/mechanical/Vase_shape.png)
Type "s" to access the "search menu" and enter "revolve" to see the tool. You'll have to select the shape and the axis you want and press "enter".
![img](../img/mechanical/vase_glass.png)

### Appearance
To make design easier for others to imagine or simply to choose colors and material purely esthetically, the "Appearance" tool is a must! You can archive more or less the same result with the "Physical Material" tool, but this is more often used for simulation. You can also combine those tools to get for example a steel pipe covered in leather without having to create the wrapping in a component.
It is also very handy when creating complex assemblies with little pieces because the different appearance help to see those different pieces.

Appearance is really easy to use. Type "a", choose the look you like and drag-drop it on the body you want. You can directly drop it in the project tree or on the visual objet.  
![img](../img/mechanical/Appearance_tool.png)

### Pattern
Real time savior, "Pattern" allows you to duplicate sketches, bodies and components on a given distance.
Multiple pattern tools exists. To see then go into the "search menu" and type "Pattern".
![img](../img/mechanical/multiple_patterns.png)
As you can see in the above picture, you can create a circular, rectangular or "free style" pattern.
For both rectangular and circular patterns the ones with white square create pattern in sketches. The grey ones create patterns of volumes (bodies, components). We'll only present the sketch pattern, but the volumes work in a similar way.
#### Rectangular pattern
![img](../img/mechanical/Pattern_open.png)
To apply rectangular patterns, select the desired shape and the amount of copies you want in the 2 directions. You can then enter (or drag-drop) the distance on which you want to copy that shape for both directions and click "ok".
![img](../img/mechanical/Rect_patt_1side.png)
![img](../img/mechanical/Rect_patt_2side.png)
![img](../img/mechanical/Rect_pattern_done.png)
#### Circular pattern
Circular patterns work in a similar way accept the fact you have to choose a center point and you can choose to make a complete revolution around it you on a given angle.
![img](../img/mechanical/circ_patt-full.png)
![img](../img/mechanical/circ_patt_angle.png)

*Note that it's also possible to draw a path and then follow it with our pattern.*


### Mirror
Available in sketch and also in 3D, the "Mirror" tool is very useful to create quickly two times the same things symmetrically from a line or a plane.

For the 3D, begin draw your piece, in this case, a notch to close a wall.
![img](../img/mechanical/Mirror_1.png)

Use the Mirror tool. Select the faces that you want to copy and then the plane.
![img](../img/mechanical/Mirror_2.png)
*Note that if you select only the top face of your notch you won't be able to copy it because the program doesn't allow you to create a new face in the empty space. You have to select all the three faces of your notch.*
![img](../img/mechanical/Mirror_3.png)

As We said, it's also available for Sketch mode. So if we want to make two holes symmetric. Just draw a circle ("c" key). Draw a construction line that we'll use as our center line. ("x" key and then "l" key). Chose the distance from the center line ("d" key).Then select "Mirror", select the circle as "Object" and in the "Mirror line" select the construction line
![img](../img/mechanical/Mirror_4.png)
![img](../img/mechanical/Mirror_5.png)

We can now make our two holes. Select the two of them (with "ctrl") and then cut them ("q" key).
![img](../img/mechanical/Mirror_6.png)

### Fillet
As you used it in Autocad or similars, Fillet allow you to joint two lines to make one curve.
In Fusion 360, as a 3D software, you'll also find the possibility to "curve" your 3D model.
![img](../img/mechanical/Fillet_1.png)
<figure>
  <figcaption>Fig. - First Fillet is for sketck and the second one for 3D model.</figcaption>
</figure>

To use the sketch fillet, just select it and then click on the two lines to join.
![img](../img/mechanical/Fillet_2.png)

In the 3D model, just select the ridge and with the arrow or value chose the size of your fillet.
![img](../img/mechanical/Fillet_3.png)
![img](../img/mechanical/Fillet_4.png)

### Chamfer
In the same way of thinking than for the Fillet, you'll have the possibility to make some Chamfer.
![img](../img/mechanical/Chamfer_1.png)
Select the tools and click on the ridge.
![img](../img/mechanical/Chamfer_2.png)
You'll also be able to do it with a curved ridge (the one made with fillet)
![img](../img/mechanical/Chamfer_3.png)
![img](../img/mechanical/Chamfer_4.png)
*Note that in this case you'll be limited by the angle of the previous made fillet.*

Another option is to change the "Chamfer type" and select "Distance and angle" to make a chamfer with a selected angle.
![img](../img/mechanical/Chamfer_5.png)


### Join
A joint is a link between two pieces that describes the way they move one on the other. Is comes in handy when describing actions to external people, or when modeling pieces in an assembly to avoid collisions due to design mistakes.
To create joints, you have to first **ground components**. By grounding components you avoid them to move, and so you can use them as a static reference. Take *Cortex* for example, the base has been grounded and all the rest around him has been jointed.
![img](../img/mechanical/cortex.png)
This means you can not drag it around. The second thing you have to do is **create rigid groups**. By searching for "rigid group" when hitting the "s" key you will enter the "rigid group" menu. You can now select all the component that should move together. You'll still be able to drag them around, but only as one group and not separately anymore. This is for example the case for the mecanum wheels as you can not move is different pieces separately.
After this is done, we can now **join modules together** ('j' key). To do so select the points of the modules that will touch after jointing starting with the one you want to be able to move.
![img](../img/mechanical/joint_face1.png)![img](../img/mechanical/joint_face2.png)

After this is done you can select the type of joint you want and the axis along which to movement will have to be.

![img](../img/mechanical/joint_type.png)![img](../img/mechanical/joint_defined.png)
After confirming the settings, you can right-click on the joint to edit the joints limits and inverse the natural rotation.
![img](../img/mechanical/joint_limit.png)

## 3. Importance: dif bodies vs components       PUISS
      attention séléction du bon compo quand nouveau sketch
      link d'un dessin vers un autre

## 4. modification de fichier + stl vers 3D      PUISS


## 6. export du design vers plan et/ou slicer    WILL + PUISS

## 7. spécificité pour impression 3D     PUISS
              sens de fibres
              penser support et nettoyage de support
              penser face sur le bed
              combine de dif elem pour impression monobloc

## 8. parametre d'impression et défaut d'impression      PUISS

## 9. tour de marché sur les différentes imprimantes sur le marché      PUISS + WILL


## Contributor
- Puissant Baeyens Victor, 12098, [MisterTarock](https://github.com/MisterTarock)
- De Decker William, 14130, [WilliamHdd](https://github.com/WilliamHdd)
