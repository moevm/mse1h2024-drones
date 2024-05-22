import sys

# position computing functions
def compute_segment_index(curve, time):
    segment_index = 0
    while segment_index + 1 < len(curve) \
            and time > curve[segment_index + 1][0]:
        segment_index += 1
    return segment_index

def compute_position(curve, time):
    segment_index = compute_segment_index(curve, time)
    if segment_index == len(curve) - 1:
        return curve[-1][1]
    
    start = curve[segment_index][1]
    end = curve[segment_index + 1][1]
    
    time -= curve[segment_index][0]
    time_segment = curve[segment_index + 1][0] - curve[segment_index][0]
    t = time / time_segment
    
    return (1 - t) * start + t * end

class PlaygroundSupervisor:

    TIME_STEP = 32
    NAME = 'MARKER'

    X_CURVE = [
        (0, -3)
    ]
    Y_CURVE = [
        (0, -0.08), (40, -2), (80, -0.08), (120, -2)
    ]
    Z = 0.1

    def init(self, webots_node, properties):
        self.__supervisor = webots_node.robot
        
        self.__marker = self.__supervisor.getFromDef(PlaygroundSupervisor.NAME)
        if self.__marker is None:
            raise Exception(f'node {PlaygroundSupervisor.NAME} not found')
        self.__trans_field = self.__marker.getField('translation')
    
    def step(self):
        position = self.__trans_field.getSFVec3f()
        time = self.__supervisor.getTime()
        x = compute_position(PlaygroundSupervisor.X_CURVE, time)
        y = compute_position(PlaygroundSupervisor.Y_CURVE, time)

        self.__trans_field.setSFVec3f([x, y, PlaygroundSupervisor.Z])