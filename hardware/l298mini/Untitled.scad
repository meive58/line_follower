// ============================================================================
// Suporte do motor N20 com encoder — IMPRIMIR 2x
// ============================================================================
// Montagem: o suporte vai POR BAIXO da placa, com a abertura do berço
// voltada para cima. Dois parafusos M3 atravessam as torres laterais e a
// placa (porca por cima ou por baixo). O motor desliza pela traseira e fica
// travado entre o berço e a placa; as abraçadeiras (zip ties) passam pelos
// slots do piso e prendem o corpo do motor.
// A traseira é ABERTA para dar espaço à plaquinha do encoder e aos fios.

include <params.scad>

motor_w  = 12.4;  // largura do N20 (12 mm) + folga
motor_h  = 10.4;  // altura do N20 (10 mm) + folga
cradle_l = 18;    // comprimento do berço (cobre a caixa de redução)
front_t  = 2.2;   // parede frontal (lado do eixo da roda)
boss_d   = 4.8;   // furo para o ressalto ao redor do eixo (~4 mm)
base_t   = 3;     // espessura do piso
side_t   = 2.4;   // paredes laterais do berço
tower_w  = 9;     // torres laterais que recebem os parafusos M3
ziptie_w = 4.5;   // largura do slot da abraçadeira

total_l = front_t + cradle_l;
inner_w = motor_w + 2 * side_t;
total_w = inner_w + 2 * tower_w;
total_h = base_t + motor_h;

module n20_mount() {
    difference() {
        cube([total_l, total_w, total_h]);

        // cavidade do motor (aberta atrás e em cima)
        translate([front_t, tower_w + side_t, base_t])
            cube([cradle_l + 1, motor_w, motor_h + 1]);

        // furo do ressalto do eixo na parede frontal
        translate([-1, total_w / 2, base_t + motor_h / 2])
            rotate([0, 90, 0])
                cylinder(d = boss_d, h = front_t + 2);

        // furos M3 verticais nas torres (parafuso atravessa até a placa)
        for (y = [tower_w / 2, total_w - tower_w / 2])
            translate([total_l / 2, y, -1])
                cylinder(d = m3_hole, h = total_h + 2);

        // slots de abraçadeira no piso, junto às paredes internas
        for (y = [tower_w + side_t + 0.6,
                  tower_w + side_t + motor_w - 3.6])
            translate([front_t + cradle_l / 2 - ziptie_w / 2, y, -1])
                cube([ziptie_w, 3, base_t + 2]);
    }
}

n20_mount();
