'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (keyframeNames, times, keys)
    keyframeNames := [str, ...]  # list of joint keyframeNames
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import topple
from keyframes import leftBackToStand



class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamkeyframeName='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamkeyframeName, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.firstTime = 0 #sets the time to 0
        self.lastTime = 0 #set lastTime to 0    	

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        
        loopKeyframes = True

        (keyframeNames, times, keys) = keyframes
        self.currentTime = self.perception.time

        if self.lastTime == 0:
            for keyframeNameIndex in range(0, len(keyframeNames)):
                keyframeName = keyframeNames[keyframeNameIndex]
                timesForkeyframeName = times[keyframeNameIndex]
                keysForkeyframeName = keys[keyframeNameIndex]
                if timesForkeyframeName[len(timesForkeyframeName)-1]> self.lastTime:
                   self.lastTime = timesForkeyframeName[len(timesForkeyframeName)-1]


        timeInKeyframes = self.currentTime - self.firstTime

        if self.lastTime < timeInKeyframes and loopKeyframes:
            self.firstTime = 0;

        if self.firstTime == 0:
            self.firstTime = self.currentTime
            timeInKeyframes = self.currentTime - self.firstTime


        for keyframeNameIndex in range(0, len(keyframeNames)):
            #iterare through all joints
            keyframeName = keyframeNames[keyframeNameIndex]
            timesForkeyframeName = times[keyframeNameIndex]
            keysForkeyframeName = keys[keyframeNameIndex]
            
            if timeInKeyframes < timesForkeyframeName[0]:
                '''
                before the first keyframe
                take the left handle of the first keyframe as second handle of the current position

                '''
                if not keyframeName in self.perception.joint:
                    continue

                point0 = (timeInKeyframes, self.perception.joint[keyframeName])
                point1 = (timesForkeyframeName[0] + keysForkeyframeName[0][1][1], keysForkeyframeName[0][0] + keysForkeyframeName[0][1][2])
                point2 = (timesForkeyframeName[0], keysForkeyframeName[0][0]) 
                point3 = (timesForkeyframeName[0] + keysForkeyframeName[0][1][1], keysForkeyframeName[0][0] + keysForkeyframeName[0][1][2])

                t = (timeInKeyframes - self.firstTime) / (timesForkeyframeName[0] - self.firstTime)
                k = 1

                #check float errors
                if t > 1:
                    t = 1.0

                target_joints[keyframeName] = ((k-t) ** 3) * point0[1] + 3 * ((k-t) ** 2) * t * point1[1] + 3 * (k-t) * (t ** 2) * point2[1] + (t**3) * point3[1]

            else:

                if timeInKeyframes >= timesForkeyframeName[-1]:
                    continue

                i = 0
                while timeInKeyframes > timesForkeyframeName[i]:
                    i += 1

                i = i - 1

                point0 = (timesForkeyframeName[i], keysForkeyframeName[i][0])
                point1 = (timesForkeyframeName[i] + keysForkeyframeName[i][2][1], keysForkeyframeName[i][0] + keysForkeyframeName[i][2][2])
                point2 = (timesForkeyframeName[i+1], keysForkeyframeName[i+1][0]) #
                point3 = (timesForkeyframeName[i+1] + keysForkeyframeName[i+1][1][1], keysForkeyframeName[i+1][0] + keysForkeyframeName[i+1][1][2]) 

                t = (timeInKeyframes - timesForkeyframeName[i]) / (timesForkeyframeName[i+1] - timesForkeyframeName[i])
                k = 1

                #check float errors
                if t > 1:
                    t = 1.0

                target_joints[keyframeName] = ((k-t) ** 3) * point0[1] + 3 * ((k-t) ** 2) * t * point1[1] + 3 * (k-t) * (t ** 2) * point2[1] + (t**3) * point3[1]
                

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
