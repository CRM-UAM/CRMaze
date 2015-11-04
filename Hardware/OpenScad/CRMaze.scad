// CRMaze robot
// By Víctor Uceda & Carlos García
//   Club de Robótica-Mecatrónica,
//   Universidad Autónoma de Madrid
//   http://crm-uam.github.io
//
// Derived from:
//   Pololu motor STL by Andrew Taylor ( https://grabcad.com/library/pololu-micro-motor-1-250-reduction-1 )
//   Sharp IR sensor STL by Mojtaba ( https://grabcad.com/library/ir-sensor )


// Increase the resolution of default shapes
$fa = 5; // Minimum angle for fragments [degrees]
$fs = 0.5; // Minimum fragment size [mm]


largo=75;
ancho=70;
grosor=1.6;

tolerancia=0.225;

module rueda(ra=10,ancho=17){
    translate([-6,-5,1]) import("libs/motor.stl");

    color("gray") union() {
        translate([0,0,19])
            difference() {
                hull() {
                    cylinder(r=ra,h=ancho-2);
                    translate([0,0,ancho-2.01]) cylinder(r=ra-1,h=2);
                }
                difference() {
                    cylinder(r=3.3/2+tolerancia,h=ancho+1);
                    translate([-10,1.2+tolerancia,0]) cube([20,20,ancho+2]);
                }
                translate([0,0,ancho+1]) sphere(r=5/2);
                translate([0,0,-0.1]) cylinder(r=8.5,h=9);
            }
    }
   
}

module sharp(){
    scale(10)import("libs/sharp.stl");
}
module bateria(){
    color("green") cube([58,15,30],center=true);
}

module MPU6050(){
   color("blue")cube([4,4,0.9]);
}

module L293(){
    color("black")cube([20,7.1,3],center=true);
}

//color("red")translate([65,0,grosor*2+6])cube([17.8,33,2],center=true);
*translate([largo/2.5,0,0])
    difference(){
        cube([largo,ancho,grosor],center=true);
        translate([-largo/2,ancho/3,0])cube([largo/2.3,ancho/2.3,20],center=true);
        translate([-largo/2,-ancho/3,0])cube([largo/2.3,ancho/2.3,20],center=true);
    }


module pcb() {
    union() {
        translate([-5,0,0])
            cylinder(r=15/2,h=grosor,center=true);
        
        cube([10,35,grosor],center=true);

        translate([20,0,0])
            cube([40,15,grosor],center=true);

        hull() {
        translate([12,0,0])
            cube([0.1,ancho,grosor],center=true);
        /*translate([largo/1.5,0,0])
            cube([0.1,ancho,grosor],center=true);
        translate([largo,0,0])
            cube([0.1,15,grosor],center=true);*/
        translate([largo-ancho/2,0,0])
            difference() {
                cylinder(r=ancho/2,h=grosor,center=true,$fn=40);
                translate([-ancho/2,0,0])
                    cube([ancho,ancho,grosor*2],center=true);
            }
        }
    }
}

translate([0,0,6]) {
    rotate([90,0,0]) rueda();
    rotate([-90,0,0]) rueda();
}

//translate([largo/1.3+largo/9,-ancho/2,30+6])rotate([0,90,0])sharp();
//translate([largo/1.3-largo/9,ancho/2,30+6])mirror([0,1,0])rotate([0,90,0])sharp();
//translate([-10,15,30+6+8])rotate([0,0,-90])sharp();
   
translate([35,0,16]) bateria();
//translate([50,-2,7]) MPU6050(); 
//translate([30,-25,7]) L293(); 

//!projection(cut=true)
    pcb();


dTornillo=3;
xSharp=8;
ySharp=10.5;
!difference(){
union() {
    translate([-13.5,-36.5,35]) linear_extrude(height=2) {
        //import("laberinto-B_Cu.dxf");
        hull() import("laberinto-Edge_Cuts.dxf");
    }
   translate([-8.5,-0.2,0])difference(){
        cylinder(d=6.5,h=35);
        translate([0,0,-20])cylinder(d=dTornillo,h=30);
    }
    translate([41.5,-28.5,0])difference(){
        cylinder(d=6.5,h=35);
        translate([0,0,-20])cylinder(d=dTornillo,h=30);
    }
    translate([57.2,15.3,0])difference(){
        cylinder(d=6.5,h=35);
        translate([0,0,-20])cylinder(d=dTornillo,h=30);
    }
}
    translate([60,0,0])cylinder(r=10,h=100,$fn=5);
    translate([38,22,35.7])rotate([0,0,-90])linear_extrude(15)scale(0.8)text("CRMaze", font="Liberation Sans:style=Bold");
    for(i=[0:4]){
        translate([8*i-3,0,0])cube([3,8.5,100],center=true);
    }
    
    translate([34,-25,35])rotate([0,0,45])cube([xSharp,ySharp,10],center=true); 
    translate([34,25,0])rotate([0,0,-45])cube([xSharp,ySharp,100],center=true);
    translate([18,-25,0])rotate([0,0,45])cube([xSharp,ySharp,100],center=true);
    translate([18,25,0])rotate([0,0,-45])cube([xSharp,ySharp,100],center=true);
}