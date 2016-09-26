/**
* Código para construir laberinto modular con celdas 15*15cm y paredes de grosor 2c
*/


/**
* Columna de altura h con ranuras desde la parte superior hasta una altura h_ranuras (<h)
*/
module columna(g=1.2, h=70, h_ranuras=28){
    difference(){
        translate([0,0,h/2 ])cube([20,20,h],center=true);
        translate([0,0,h/2+g])cube([20-2*g,20-2*g,h],center=true); 
        
        for(i=[0:4]){
            rotate([0,0,90*i])union(){
            translate([10,3.5,h/2 + h_ranuras/2])cube([4+0.4,g*1.2,h-h_ranuras],center=true);
            translate([10,-3.5,h/2 + h_ranuras/2])cube([4+0.4,g*1.2,h-h_ranuras],center=true);
            }
        }
    }
}

/**
* Pared de altura h con inserciones desde la parte superior hasta una altura h_ranuras (<h)
*/
module pared(g=1.2, h=70, h_ranuras=30){
    translate([65,-5,h/2])cube([128,g,h],center=true);
    
    translate([1+0.6,-1.5,h/2])cube([g,7,h],center=true);
    translate([0,1.5,h/2 + h_ranuras/2])cube([4+0.4,g,h-h_ranuras],center=true);
    translate([-2-0.3,1.5,h_ranuras])cylinder(r=g*0.9,h=h-h_ranuras, $fn=20);

    translate([-1-0.6+130,-1.5,h/2])cube([g,7,h],center=true);
    translate([130,1.5,h/2+h_ranuras/2])cube([4+0.6,g,h-h_ranuras],center=true);
    translate([130+2+0.3,1.5,h_ranuras])cylinder(r=g*0.9,h=h-h_ranuras, $fn=20);
}

*union(){ //bloque de paredes y columnas coladas en la disposición final
%columna();
translate([0,150,0])columna();
translate([150,0,0])columna();

translate([10,-5,0])pared();
translate([140,5,0])rotate([0,0,180])pared();
translate([5,10,0])rotate([0,0,90])pared();
%translate([5,-10-130,0])rotate([0,0,90])pared();
}

module bloque_paredes(dim=10, dim2=4){
    for(i=[0:dim-1]){
        translate([0,i*13,0])pared();
        translate([120,i*13,0])rotate([0,0,180])pared();
    }
    for(i=[0:dim2-1]){
        translate([139+i*13,25,0])rotate([0,0,90])pared();
        translate([139+i*13,140,0])rotate([0,0,-90])pared();
    }
}

module bloque_columnas(dim=10){
    for(i=[0:dim-1]){
        translate([0,i*23,0])union(){
            for(j=[0:dim-1])
                translate([j*23,0,0])columna();
        }
    }
}


*bloque_columnas(8); 
bloque_paredes(14, 4);
%translate([-10,-10,-10])cube([198,198,5]); // base de impresión