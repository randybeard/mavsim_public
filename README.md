# uavbook
Repository for the textbook:  Small Unmanned Aircraft: Theory and Practice, by Randy Beard and Tim McLain

![Bookcover](./bookcover.jpg)

[R. Beard](http://www.ee.byu.edu/faculty/beard/), 
[T. McLain](http://me.byu.edu/faculty/timmclain)

[Princeton University Press, 2012](http://press.princeton.edu/titles/9632.html)

[Small Unmanned Aircraft: Theory and Practice: Supplement](files/uavbook_supplement.pdf)
  * Supplemental material is currently under construction.  It will be regularly updated during Winter 2022.  This file is preliminary and has not been carefully proof read.

## LECTURE MATERIAL

The following lecture materials are included as a resource for instructors.  The slides closely follow the book.  We welcome suggestions on how these slides might be improved. - RWB & TWM

| Chapter | PDF Slides | Powerpoint | Last Modified |
|---------|------------|------------|----------|
| Chapter 1 - Introduction | [chap1.pdf](lecture/chap1.pdf) | [chap1.pptx](lecture/chap1.pptx) | 9/10/2014  |
| Chapter 2 - Coordinate Frames | [chap2.pdf](lecture/chap2.pdf) | [chap2.pptx](lecture/chap2.pptx) | 1/11/2019  |
| Chapter 3 - Kinematics and Dynamics | [chap3.pdf](lecture/chap3.pdf) | [chap3.pptx](lecture/chap3.pptx) | 1/14/2022 |
| Chapter 4 - Forces and Moments | [chap4.pdf](lecture/chap4.pdf) | [chap4.pptx](lecture/chap4.pptx)  | 1/21/2022 |
| Chapter 5 - Linear Design Models | [chap5.pdf](lecture/chap5.pdf) | [chap5.pptx](lecture/chap5.pptx)  | 2/10/2022 |
| Chapter 6 - Autopilot Design | [chap6.pdf](lecture/chap6.pdf) | [chap6.pptx](lecture/chap6.pptx)  | 2/10/2022 |
| Chapter 7 - Sensors | [chap7.pdf](lecture/chap7.pdf) | [chap7.pptx](lecture/chap7.pptx)  | 2/14/2022 |
| Chapter 8 - State Estimation | [chap8.pdf](lecture/chap8.pdf) | [chap8.pptx](lecture/chap8.pptx)  | 2/28/2020 |
| Chapter 9 - Nonlinear Design Models | [chap9.pdf](lecture/chap9.pdf) | [chap9.pptx](lecture/chap9.pptx)  | 11/4/2014 |
| Chapter 10 - Waypoint and Orbit Following | [chap10.pdf](lecture/chap10.pdf) | [chap10.pptx](lecture/chap10.pptx)  | 3/25/2020 |
| Chapter 11 - Path Manager | [chap11.pdf](lecture/chap11.pdf) | [chap11.pptx](lecture/chap11.pptx)  | 03/30/2020 |
| Chapter 12 - Path Planning | [chap12.pdf](lecture/chap12.pdf) | [chap12.pptx](lecture/chap12.pptx)  | 04/03/2017 |
| Chapter 13 - Cameras| [chap13.pdf](lecture/chap13.pdf) | [chap13.pptx](lecture/chap13.pptx)  | 04/10/2017 |

## PROJECT FILES 

The following files are included to help students with the project outlined in the book.  We have found that if students start with these files, that they can generally do the project in about 3 hours per chapter.  Full solutions to the project are available to instructors upon request.  We respectfully ask that students and instructors do not post full solutions to the project anywhere on the web.  The project creates an excellent learning experience and we believe that anyone who works the project for themselves will be much better equipped to make contributions to the state of the art in small unmanned air vehicles. - RWB & TWM//

## Github repository
The template files for Simulink, Matlab OOP, and Python are available at the following github repository:

[[https://magiccvs.byu.edu/gitlab/uavbook/mavsim_template_files]]

## Comments
  * There are some issues with the Zagi coefficients given in the book.  We recommend that you use the following (slightly modified from the book) coefficients for the aerosonde aircraft. [aerosonde.m](project/aerosonde.zip)
  * For the Aerosonde model use an initial speed of Va = 35 m/s, and and radius of 250 meters.
  * Additional information: The Aerosonde weight is actually 25kg.  Also, the cruise speed is approximately Va=35 m/s.

## Video Solutions 
* [YouTube video showing solution for Chapter 2.](http://youtu.be/LgiHUznfP_4)
* [YouTube video showing solution for Chapter 3.](http://youtu.be/KCoRO-G-VPg)
* [YouTube video showing solution for Chapter 4.](http://youtu.be/jT5_ZDyNCTI)
* [YouTube video showing solution for Chapter 5.](http://youtu.be/cyLi8WAbOWs)
* [YouTube video showing solution for Chapter 6.](http://youtu.be/1CoF2rJHs4c)
* [Screen shot of estimation results](project/chap8_results.zip)
* [YouTube video showing solution for Chapter 10.](http://youtu.be/1NEssGinf_8)
* [YouTube video showing solution for Chapter 11.](http://youtu.be/aGAdjbtSoso)
* [YouTube video showing solution for Chapter 12.](http://youtu.be/bdYb45bpID4)

## ERRATA

As you find typos and errors in the book, please send us an email.  We will post the errors on this page and work with Princeton University Press to ensure that they do not propagate to future printings.  - RWB & TWM

All corrections and additions in the supplement will be incorporated into a future second edition of the book.

[UAVbook Supplement](files/uavbook_supplement.pdf)
  

## SUPPLEMENTAL MATERIAL

Our intention is to occasionally add supplemental material to this page.  We would also welcome contributions from the broader community.  If you are interested in adding material, please contact the authors. 

  * **Adaptation of MavSim to Underwater Vehicles**
    * [Underwater simulator](https://bitbucket.org/jfarrant/python_fixedwing_mavsim)
    * by James Farrant 

  * **Full Longitudinal State Direct and Indirect Kalman Filter**
    * The Kalman filters presented in the book are meant to be tutorial and are intended for aircraft with very limited processors.  Estimating the full state using all available sensors is a much better approach.  There are two methods for constructing the Kalman filter: direct state estimation, and indirect state estimation.  The following pdf and Simulink model describes and implements both filters for the full longitudinal state.  You will note from the simulation, that this method works much better than that described in the book.  
    * {{:shared:ekf_full_state.pdf|UAVBook Supplement on Kalman Filtering.}} (updated 3/15/2017)
    * {{:shared:mavsim_simulation.zip|Simulation of both filters for the Longitudinal States.}}

  * **Dropping an Object on a Target**
    * {{:shared:object_drop.pdf|UAVBook Supplement on Object Dropping.}} (updated 10/25/2017)

  * **Dubins Airplane Paths**
    * The Dubins paths discussed in Chapter 11 assume that the MAV is flying at a constant altitude.  The associated model is typically called a Dubins car model.  The Dubins car can be extended to a Dubins airplane model that includes altitude.  An explanation of the associated Dubins airplane paths is discussed in:
      * Mark Owen, Randal W. Beard, Timothy W. McLain, “Implementing Dubins Airplane Paths on Fixed-wing UAVs,” Handbook of Unmanned Aerial Vehicles, ed. Kimon P. Valavanis, George J. Vachtsevanos, Springer Verlag, Section XII, Chapter 68, p. 1677-1702, 2014. {{:shared:dubins_airplane_paths.pdf|Preprint.}}
    * {{:shared:dubins_airplane_simulation.zip|Simulink files implementing Dubins airplane paths.}}  Run the Simulink file mavsim_dubins.slx.


  * **Feedforward term for Orbit Following**
    * Orbit following can be improved by adding a feedforward term on the roll angle.   The description in the wind and no wind case are given here:
    * {{:shared:feedforward_orbit_following.pdf|UAVBook Supplement on Feedforward for Orbit Following.}}

  * **Python Simulator**
    * A simulator that implements the architecture in the book using Python
    * PyUAVSim at https://github.com/savadha2/PyUAVSim/
    * Developed by Sharath Avadhanam.
    * Currently under development.

  * **Zagi Coefficients**
    * {{:shared:zagi_coefficents.pdf|}}  The aerodynamic coefficients for the Zagi aircraft as given in the book, come from this paper.

  * **Total Energy Control**
    * An alternative to the longitudinal autopilot described in the textbook is the "Total Energy Control System" described here: {{:shared:tecs_autopilot.pdf|}}.
    * A nonlinear version of the total energy control system is described in the following paper.
      * Matthew Argyle, Randal W. Beard, “Nonlinear Total Energy Control for the Longitudinal Dynamics of an Aircraft,” Proceedings of the American Control Conference, Boston, MA, 2016.{{:shared:ArgyleBeard16.pdf|PDF}}
    * For more detail, see Chapter 6 of 
      * Matthew E. Argyle, “Modeling and Control of a Tailsitter with a Ducted Fan,” PhD Dissertation, Brigham Young University, 2016.  {{:shared:Argyle_chap6.pdf|PDF}}
    * The advantage of the total energy control method is that it is independent of the aerodynamic model.

  * **Accelerometers and Attitude Control For Multi-rotors**
    * Accelerometers are often used to estimate the roll and pitch angles of multi-rotor vehicles.  It turns out that since the aerodynamics of multi-rotors are quite different than fixed wing vehicles, the method described in the book does not work for multi-rotor vehicles.  An detailed explanation of what data can be extracted from accelerometers on multi-rotor vehicles is given in 
      * Robert Leishman, John Macdonald, Randal W. Beard, Timothy W. McLain, “Quadrotors and Accelerometers: State Estimation with an Improved Dynamic Model,”  IEEE Control Systems Magazine, vol. 34, no. 1, p. 28-41, February, 2014. {{:shared:csm-13-0006_v4.pdf|Preprint}}.

  * **Kalman Filter**
    * For a nice introduction to Kalman filtering, view the section chapter of the Udacity course on Artificial Intelligence by Sebastian Thurn:
    * [[https://www.udacity.com/course/viewer#!/c-cs373/l-48723604]]

  * **Weather and Wind Data**
    * This is a nice web site for hour by hour wind and weather information:
    * [[http://www.usairnet.com/cgi-bin/launch/code.cgi?Submit=Go&sta=KPVU&state=UT]]

  * **Student Projects (sample)**
    * {{:shared:2014_nathan_madsen_auto_landing.zip|}} - Nathan Madsen implemented a model of landing gear and runway interactions, together with an auto landing function.
    * {{:shared:andrew_hendricks_avl_aero_coefficients.zip|}} - Andrew Hendrick computed the aerodynamic coefficients of a Zagi and a Pelican using AVL.  The project files and the coefficients are contained in this zipped file.  {{:shared:andrew_hendricks_avl_aero_coefficients.pdf|}}
    * {{:shared:2014_rrtstar_michael_boren.zip|}} - Michael Boren's implementation of the RRT* algorithm.
    * 2019.  Michael Eyler: LQR control: [[https://github.com/eyler94/EE674LQR.git]]
    * 2019. Brandon Forsgren.  ROS/C++ implementation: [[https://github.com/b4sgren/mav_simulator]]
    * 2019. Derek Knowles. Path planning Voronoi graphs: [[https://github.com/betaBison/EC-EN-674-Flight-Dynamics-Controls/tree/master/project]]




