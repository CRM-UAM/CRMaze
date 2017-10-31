// Autor: Victor Uceda Uceda
// Licencia: Public Domain

module columna(){
    translate([0,0,40])cube([10,10,80], center=true);
}

module pinza2paredes(){
difference(){
translate([0,0,12.5])cube([13.5,40,25],center=true);
translate([0,0,14])cube([10.5,60,25],center=true);
}
}

module pinza4paredes(){
difference(){
    union(){
        rotate([0,0,90])translate([0,0,12.5])cube([13.5,30,25],center=true);
        translate([0,0,12.5])cube([13.5,30,25],center=true);
    }
    
    translate([0,0,14])cube([10.5,60,25],center=true);
    rotate([0,0,90])translate([0,0,14])cube([10.5,60,25],center=true);
}
}
module pinza3paredes(){
difference(){
    union(){
        rotate([0,0,90])translate([0,0,12.5])cube([13.5,40,25],center=true);
        translate([0,0,12.5])cube([13.5,40,25],center=true);
    }
    
    translate([0,0,14])cube([10.5,60,25],center=true);
    rotate([0,0,90])translate([0,15,14])cube([10.5,30,25],center=true);
    translate([13.5/2,-25,-10])cube([20,50,50]);
}
}


//columna();
//pinza4paredes();

for (i = [-30:30])
    for (j = [-30:30]) {
        x = i*14.3+j*31;
        y = i*22.5;
        maxLen = 170/2;
        if( x > -maxLen && x < maxLen-12 && y > -maxLen && y < maxLen)
        translate([x,y,0]) {
            columna();
            pinza4paredes();
        }
    }
%cube([190,190,1],center=true);

/*
translate([50,0,0])columna();
translate([50,0,0])pinza2paredes();


translate([-50,0,0])columna();
translate([-50,0,0])pinza3paredes();
*/