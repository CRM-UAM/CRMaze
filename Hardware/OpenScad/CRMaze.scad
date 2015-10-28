largo=75;
ancho=70;
grosor=1.6;

module rueda(ra=10,ancho=20){
    translate([-6,-5,1]) import("motor.stl");

    color("gray") union() {
        translate([0,0,19]) difference() {
            cylinder(r=ra,h=ancho-3);
            translate([0,0,-ancho/2]) cylinder(r=ra*0.8,h=ancho*2);
        }
        difference() {
            translate([0,0,29]) cylinder(r=ra,h=2);
            translate([0,0,28]) cylinder(r=2,h=5,$fn=20);
        }
    }
   
}

module sharp(){
    scale(10)import("sharp.stl");
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
            cylinder(r=15/2,h=grosor,center=true,$fn=20);
        
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
   
translate([35,0,15]) bateria();
//translate([50,-2,7]) MPU6050(); 
//translate([30,-25,7]) L293(); 

//!projection(cut=true)
    pcb();
