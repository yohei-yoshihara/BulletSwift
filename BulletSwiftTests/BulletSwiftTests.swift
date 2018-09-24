/**
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 
 Swift Binding
 Copyright (c) 2018 Yohei Yoshihara
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software.
 If you use this software in a product, an acknowledgment in the product documentation would be appreciated
 but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

import XCTest
@testable import BulletSwift

class MotionState : BulletMotionState {
  var transform = BulletTransform()
  init(transform: BulletTransform) {
    self.transform = transform
  }
  func getWorldTransform() -> BulletTransform {
    return transform
  }
  
  func setWorldTransform(centerOfMassWorldTransform: BulletTransform) {
    self.transform = centerOfMassWorldTransform
  }
}

class BulletSwiftTests: XCTestCase {

    override func setUp() {
    }

    override func tearDown() {
    }

  func test_HelloWorld() {
    let world = BulletWorld()
    world.gravity = vector3(0, -10, 0)
    
    let groundShape = BulletBoxShape(halfExtents: vector3(50, 50, 50))
    
    let groundTransform = BulletTransform()
    groundTransform.setIdentity()
    groundTransform.origin = vector3(0, -56, 0)
    
    let groundMotionState = MotionState(transform: groundTransform)
    let groundBody = BulletRigidBody(mass: 0,
                                     motionState: groundMotionState,
                                     collisionShape: groundShape)
    world.add(rigidBody: groundBody)
    
    let colShape = BulletSphereShape(radius: 1.0)
    let startTransform = BulletTransform()
    startTransform.setIdentity()
    let mass: Float = 1.0
    let localInertia = colShape.calculateLocalInertia(mass: mass)
    
    startTransform.origin = vector3(2, 10, 0)
    
    let colMotionState = MotionState(transform: startTransform)
    
    let colBody = BulletRigidBody(mass: mass,
                                  motionState: colMotionState,
                                  collisionShape: colShape,
                                  localInertia: localInertia)
    world.add(rigidBody: colBody)
    
    for i in 0 ..< 150 {
      world.stepSimulation(timeStep: 1.0/60.0, maxSubSteps: 10, fixedTimeStep: 1.0/60.0)
      print("\(colMotionState.transform.origin)")
      
      let expected = vector3(expectedResult[i * 3],
                             expectedResult[i * 3 + 1],
                             expectedResult[i * 3 + 2])
      XCTAssertEqual(colMotionState.transform.origin.x, expected.x, accuracy: 1e-5)
      XCTAssertEqual(colMotionState.transform.origin.y, expected.y, accuracy: 1e-5)
      XCTAssertEqual(colMotionState.transform.origin.z, expected.z, accuracy: 1e-5)
    }
    
    world.remove(rigidBody: groundBody)
    world.remove(rigidBody: colBody)
    
  }
  
  let expectedResult: [Float] = [
    2.000000,10.000000,0.000000,
    2.000000,9.997222,0.000000,
    2.000000,9.991667,0.000000,
    2.000000,9.983334,0.000000,
    2.000000,9.972222,0.000000,
    2.000000,9.958333,0.000000,
    2.000000,9.941667,0.000000,
    2.000000,9.922222,0.000000,
    2.000000,9.900000,0.000000,
    2.000000,9.875000,0.000000,
    2.000000,9.847222,0.000000,
    2.000000,9.816667,0.000000,
    2.000000,9.783333,0.000000,
    2.000000,9.747222,0.000000,
    2.000000,9.708333,0.000000,
    2.000000,9.666666,0.000000,
    2.000000,9.622222,0.000000,
    2.000000,9.575000,0.000000,
    2.000000,9.525000,0.000000,
    2.000000,9.472221,0.000000,
    2.000000,9.416666,0.000000,
    2.000000,9.358333,0.000000,
    2.000000,9.297221,0.000000,
    2.000000,9.233333,0.000000,
    2.000000,9.166666,0.000000,
    2.000000,9.097221,0.000000,
    2.000000,9.025000,0.000000,
    2.000000,8.950000,0.000000,
    2.000000,8.872222,0.000000,
    2.000000,8.791666,0.000000,
    2.000000,8.708333,0.000000,
    2.000000,8.622222,0.000000,
    2.000000,8.533333,0.000000,
    2.000000,8.441667,0.000000,
    2.000000,8.347222,0.000000,
    2.000000,8.250000,0.000000,
    2.000000,8.150000,0.000000,
    2.000000,8.047222,0.000000,
    2.000000,7.941667,0.000000,
    2.000000,7.833333,0.000000,
    2.000000,7.722222,0.000000,
    2.000000,7.608334,0.000000,
    2.000000,7.491667,0.000000,
    2.000000,7.372222,0.000000,
    2.000000,7.250000,0.000000,
    2.000000,7.125000,0.000000,
    2.000000,6.997222,0.000000,
    2.000000,6.866667,0.000000,
    2.000000,6.733334,0.000000,
    2.000000,6.597222,0.000000,
    2.000000,6.458333,0.000000,
    2.000000,6.316667,0.000000,
    2.000000,6.172223,0.000000,
    2.000000,6.025001,0.000000,
    2.000000,5.875000,0.000000,
    2.000000,5.722223,0.000000,
    2.000000,5.566667,0.000000,
    2.000000,5.408334,0.000000,
    2.000000,5.247223,0.000000,
    2.000000,5.083334,0.000000,
    2.000000,4.916667,0.000000,
    2.000000,4.747223,0.000000,
    2.000000,4.575001,0.000000,
    2.000000,4.400001,0.000000,
    2.000000,4.222223,0.000000,
    2.000000,4.041667,0.000000,
    2.000000,3.858334,0.000000,
    2.000000,3.672222,0.000000,
    2.000000,3.483333,0.000000,
    2.000000,3.291667,0.000000,
    2.000000,3.097222,0.000000,
    2.000000,2.900000,0.000000,
    2.000000,2.700000,0.000000,
    2.000000,2.497222,0.000000,
    2.000000,2.291666,0.000000,
    2.000000,2.083333,0.000000,
    2.000000,1.872222,0.000000,
    2.000000,1.658333,0.000000,
    2.000000,1.441666,0.000000,
    2.000000,1.222221,0.000000,
    2.000000,0.999999,0.000000,
    2.000000,0.774999,0.000000,
    2.000000,0.547221,0.000000,
    2.000000,0.316665,0.000000,
    2.000000,0.083332,0.000000,
    2.000000,-0.152780,0.000000,
    2.000000,-0.391669,0.000000,
    2.000000,-0.633335,0.000000,
    2.000000,-0.877780,0.000000,
    2.000000,-1.125002,0.000000,
    2.000000,-1.375003,0.000000,
    2.000000,-1.627781,0.000000,
    2.000000,-1.883336,0.000000,
    2.000000,-2.141670,0.000000,
    2.000000,-2.402781,0.000000,
    2.000000,-2.666670,0.000000,
    2.000000,-2.933337,0.000000,
    2.000000,-3.202782,0.000000,
    2.000000,-3.475004,0.000000,
    2.000000,-3.750005,0.000000,
    2.000000,-4.027782,0.000000,
    2.000000,-4.308338,0.000000,
    2.000000,-4.591671,0.000000,
    2.000000,-4.877783,0.000000,
    2.000000,-5.133337,0.000000,
    2.000000,-5.106670,0.000000,
    2.000000,-5.085336,0.000000,
    2.000000,-5.068269,0.000000,
    2.000000,-5.054616,0.000000,
    2.000000,-5.043693,0.000000,
    2.000000,-5.034954,0.000001,
    2.000000,-5.034954,0.000001,
    2.000000,-5.027965,0.000001,
    2.000000,-5.022372,0.000001,
    2.000000,-5.017898,0.000001,
    2.000000,-5.014318,0.000001,
    2.000000,-5.011455,0.000001,
    2.000000,-5.009164,0.000001,
    2.000000,-5.007331,0.000001,
    2.000000,-5.005866,0.000001,
    2.000000,-5.004693,0.000002,
    2.000000,-5.003754,0.000002,
    2.000000,-5.003003,0.000002,
    2.000000,-5.002402,0.000002,
    2.000000,-5.001922,0.000002,
    2.000000,-5.001537,0.000002,
    2.000000,-5.001230,0.000002,
    2.000000,-5.000984,0.000002,
    2.000000,-5.000787,0.000002,
    2.000000,-5.000629,0.000003,
    2.000000,-5.000504,0.000003,
    2.000000,-5.000404,0.000003,
    2.000000,-5.000323,0.000003,
    2.000000,-5.000258,0.000003,
    2.000000,-5.000207,0.000003,
    2.000000,-5.000166,0.000003,
    2.000000,-5.000133,0.000004,
    2.000000,-5.000106,0.000004,
    2.000000,-5.000085,0.000004,
    2.000000,-5.000068,0.000004,
    2.000000,-5.000055,0.000004,
    2.000000,-5.000044,0.000004,
    2.000000,-5.000036,0.000005,
    2.000000,-5.000029,0.000005,
    2.000000,-5.000023,0.000005,
    2.000000,-5.000019,0.000005,
    2.000000,-5.000015,0.000005,
    2.000000,-5.000012,0.000006,
    2.000000,-5.000010,0.000006,
    2.000000,-5.000008,0.000006]
  
}
