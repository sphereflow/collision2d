# Collision 2D
Various functionality for basic 2D objects including:

### Objects

- Line
- Ray: Which also can refract on other objects
- LineSegment
- Circle
- MCircle:  An object representing a moving circle with velocity and weight  
            which can collide fully elastically with other MCircles
- Rect
- AABB: Axis Aligned bounding box  
        A lot of the functionality for bounding boxes, which you usually find, has not been implemented  
        or derived by converting into Rect first
- CubicBezier
- Geo: An enum for Geometric objects
- Logic: Allows the basic objects to be composed into more complex ones

### Traits

- HasGeo: Conversion into Geo
- HasOrigin: Position
- Rotate: Rotation based on nalgebras rotation matrix
- GeoT: An experiment to replace the Geo enum
- Intersect: Intersection between objects with an associated Intersection type
- ReflectOn: Reflection of objects based on the point of intersection and its normal
- Scale: Scaling objects and their position
- Contains: Wether an object contains a point
- Distance: Distance between an object and a Point
