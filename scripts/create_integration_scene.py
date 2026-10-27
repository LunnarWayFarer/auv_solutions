"""Создание учебной сцены с четырьмя движителями и виртуальными датчиками."""

from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, indent, ElementTree


def create_scene(destination):
    destination = Path(destination)
    root = Element('scenario')
    env = SubElement(root, 'environment')
    SubElement(env, 'ned', latitude='59.0', longitude='30.0')
    ocean = SubElement(env, 'ocean')
    SubElement(ocean, 'water', density='1031.0', jerlov='0.2')
    SubElement(ocean, 'waves', height='0.0')
    materials = SubElement(root, 'materials')
    SubElement(materials, 'material', name='hull', density='1031', restitution='0.1')
    table = SubElement(materials, 'friction_table')
    SubElement(table, 'friction', material1='hull', material2='hull', static='0.5', dynamic='0.3')
    looks = SubElement(root, 'looks')
    SubElement(looks, 'look', name='blue', gray='0.5', roughness='0.5')
    floor = SubElement(root, 'static', name='bottom', type='box')
    SubElement(floor, 'dimensions', xyz='100 100 1')
    SubElement(floor, 'material', name='hull')
    SubElement(floor, 'look', name='blue')
    SubElement(floor, 'world_transform', xyz='0 0 20.5', rpy='0 0 0')
    robot = SubElement(root, 'robot', name='auv_01', fixed='false', self_collisions='false')
    body = SubElement(robot, 'base_link', name='body', type='box', physics='submerged')
    SubElement(body, 'dimensions', xyz='0.8 0.4 0.3')
    SubElement(body, 'origin', xyz='0 0 0', rpy='0 0 0')
    SubElement(body, 'material', name='hull')
    SubElement(body, 'look', name='blue')
    for name, kind, rate, topic in [
        ('pressure', 'pressure', 10, '/auv_01/pressure'),
        ('imu', 'imu', 50, '/auv_01/imu'),
        ('dvl', 'dvl', 10, '/auv_01/dvl'),
        ('navigation', 'odometry', 10, '/auv_01/navigation'),
        ('truth', 'odometry', 10, '/evaluation/auv_01/ground_truth'),
    ]:
        sensor = SubElement(robot, 'sensor', name=name, type=kind, rate=str(rate))
        if kind == 'pressure':
            SubElement(sensor, 'range', pressure='1000000')
        if kind == 'imu':
            SubElement(sensor, 'noise', angle='0 0 0', angular_velocity='0',
                       linear_acceleration='0', yaw_drift='0')
        if kind == 'dvl':
            SubElement(sensor, 'specs', beam_angle='30', beam_positive_z='true')
            SubElement(sensor, 'range', velocity='10 10 10', altitude_min='0.5',
                       altitude_max='50')
        if name == 'navigation':
            SubElement(sensor, 'noise', position='0.01', velocity='0.005', angle='0',
                       angular_velocity='0')
        SubElement(sensor, 'history', samples='1')
        SubElement(sensor, 'origin', xyz='0 0 0', rpy='0 0 0')
        SubElement(sensor, 'link', name='body')
        SubElement(sensor, 'ros_publisher', topic=topic)
    for name, xyz, rpy in [
        ('horizontal_left', '0 -0.22 0', '0 0 0'),
        ('horizontal_right', '0 0.22 0', '0 0 0'),
        ('vertical_front', '0.25 0 0', '0 -1.5707963267948966 0'),
        ('vertical_rear', '-0.25 0 0', '0 -1.5707963267948966 0'),
    ]:
        actuator = SubElement(robot, 'actuator', name=name, type='thruster')
        SubElement(actuator, 'specs', max_setpoint='400', inverted_setpoint='false',
                   normalized_setpoint='true')
        prop = SubElement(actuator, 'propeller', diameter='0.12', right='true')
        SubElement(prop, 'mesh', filename='propeller.obj', scale='1.0')
        SubElement(prop, 'material', name='hull')
        SubElement(prop, 'look', name='blue')
        dynamics = SubElement(actuator, 'rotor_dynamics', type='first_order')
        SubElement(dynamics, 'time_constant', value='0.5')
        model = SubElement(actuator, 'thrust_model', type='quadratic')
        SubElement(model, 'thrust_coeff', value='0.0004')
        SubElement(actuator, 'origin', xyz=xyz, rpy=rpy)
        SubElement(actuator, 'link', name='body')
    SubElement(robot, 'ros_subscriber', thrusters='/auv_01/thruster_command')
    SubElement(robot, 'ros_publisher', thrusters='/auv_01/thruster_state')
    SubElement(robot, 'world_transform', xyz='0 0 5', rpy='0 0 0')
    indent(root)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    destination.with_name('propeller.obj').write_text(
        'v 0.01 0 0\nv -0.01 0.06 0\nv -0.01 -0.03 0.052\n'
        'v -0.01 -0.03 -0.052\nf 1 3 2\nf 1 4 3\nf 1 2 4\nf 2 3 4\n')


if __name__ == '__main__':
    create_scene(Path(__file__).resolve().parents[1]/'scenarios/integration.scn')
