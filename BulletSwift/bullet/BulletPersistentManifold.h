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

#import <Foundation/Foundation.h>
#import <simd/simd.h>
#import "BulletCast.h"

NS_ASSUME_NONNULL_BEGIN

@class BulletCollisionObject;
@class BulletManifoldPoint;

@interface BulletPersistentManifold : NSObject
@property (nonatomic, strong, readonly) BulletCollisionObject *body0;
@property (nonatomic, strong, readonly) BulletCollisionObject *body1;
@property (nonatomic, readonly) float contactBreakingThreshold;
@property (nonatomic, readonly) float contactProcessingThreshold;

- (NSUInteger)numberOfContacts;
- (NSUInteger)count;
- (BulletManifoldPoint *)contactPointAtIndex:(NSUInteger)index;
- (instancetype)initWithPersistentManifold:(btPersistentManifoldC *)pm;
@end

NS_ASSUME_NONNULL_END
