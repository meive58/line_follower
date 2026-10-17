// ============================================================================
// Visualização do layout do robô — APENAS ILUSTRATIVA (não imprimir!)
// ============================================================================
// Abra no OpenSCAD para ver onde cada componente fica na placa 80 x 80 mm.
// Origem no CENTRO da placa; +X é a FRENTE do robô; Z=0 é o topo da placa.
// Útil também para gerar uma figura do layout para o TCC (File > Export).

board   = 80;     // lado da placa
board_t = 1.6;
wheel_d = 22;     // diâmetro da roda
wheel_w = 6;
axle_x  = -25;    // eixo das rodas: 15 mm da borda traseira
track   = 96;     // distância entre os centros das rodas
axle_z  = -8;     // eixo ~8 mm abaixo da placa (motor sob a placa)

// placa do robô
color("darkgreen")
    translate([-board / 2, -board / 2, -board_t])
        cube([board, board, board_t]);

// rodas
color("dimgray")
    for (s = [-1, 1])
        translate([axle_x, s * track / 2, axle_z])
            rotate([90, 0, 0])
                cylinder(d = wheel_d, h = wheel_w, center = true);

// motores N20 (sob a placa, um apontando para cada lado)
color("silver")
    for (s = [-1, 1])
        translate([axle_x - 6, (s == 1) ? 8 : -38, axle_z - 5])
            cube([12, 30, 10]);

// ball caster dianteiro (sob a placa)
color("steelblue") {
    translate([30, -8, -board_t - 8]) cube([16, 16, 8]);
    translate([38, 0, -board_t - 10]) sphere(d = 9.5);
}

// suporte + barra do QTR-8 (na frente, sensores a ~3 mm do chão)
color("crimson") {
    translate([28, -10, -board_t - 3]) cube([34, 20, 3]);       // braço
    translate([59, -42, -board_t - 15]) cube([14, 84, 3]);      // barra
    translate([61, -38, -board_t - 17]) cube([10, 76, 2]);      // QTR-8
}

// bateria (no topo, centrada sobre/junto ao eixo das rodas)
color("orange")
    translate([-33, -30, 0])
        cube([31, 60, 14]);

// ESP32 DevKit (no topo, à frente da bateria)
color("navy")
    translate([4, -26, 0])
        cube([28.5, 52, 10]);

// MPU-6050 (idealmente no ponto médio do eixo das rodas)
color("purple")
    translate([axle_x - 10, -32, 0])
        cube([16, 16, 3]);
