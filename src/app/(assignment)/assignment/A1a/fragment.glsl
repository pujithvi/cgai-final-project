/////////////////////////////////////////////////////
//// CS 8803/4803 CGAI: Computer Graphics in AI Era
//// Assignment 1A: SDF and Ray Marching
/////////////////////////////////////////////////////

precision highp float;              //// set default precision of float variables to high precision

varying vec2 vUv;                   //// screen uv coordinates (varying, from vertex shader)
uniform vec2 iResolution;           //// screen resolution (uniform, from CPU)
uniform float iTime;                //// time elapsed (uniform, from CPU)

const vec3 CAM_POS = vec3(-0.35, 1.0, -3.0);
float sdf2(vec3 p);

struct SDFResult {
    float distance; 
    vec3 color;
};


/////////////////////////////////////////////////////
//// sdf functions
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//// Step 1: sdf primitives
//// You are asked to implement sdf primitive functions for sphere, plane, and box.
//// In each function, you will calculate the sdf value based on the function arguments.
/////////////////////////////////////////////////////

//// sphere: p - query point; c - sphere center; r - sphere radius
float sdfSphere(vec3 p, vec3 c, float r)
{
    //// your implementation starts
    
    return length(p - c) - r;
    
    //// your implementation ends
}

//// plane: p - query point; h - height
float sdfPlane(vec3 p, float h)
{
    //// your implementation starts
    
    return p.y - h;
    
    //// your implementation ends
}

//// box: p - query point; c - box center; b - box half size (i.e., the box size is (2*b.x, 2*b.y, 2*b.z))
float sdfBox(vec3 p, vec3 c, vec3 b)
{
    //// your implementation starts
    vec3 q = abs(p - c) - b;
    
    return length(max(q, 0.0)) + min(max(q.z, max(q.x, q.y)), 0.0);
    
    //// your implementation ends
}

/////////////////////////////////////////////////////
//// boolean operations
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//// Step 2: sdf boolean operations
//// You are asked to implement sdf boolean operations for intersection, union, and subtraction.
/////////////////////////////////////////////////////

float sdfIntersection(float s1, float s2)
{
    //// your implementation starts
    
    return max(s1, s2);

    //// your implementation ends
}

float sdfUnion(float s1, float s2)
{
    //// your implementation starts
    
    return min(s1, s2);

    //// your implementation ends
}

float sdfSubtraction(float s1, float s2)
{
    //// your implementation starts
    
    return max(s1, -s2);

    //// your implementation ends
}

/////////////////////////////////////////////////////
//// sdf calculation
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//// Step 3: scene sdf
//// You are asked to use the implemented sdf boolean operations to draw the following objects in the scene by calculating their CSG operations.
/////////////////////////////////////////////////////

//// sdf: p - query point
SDFResult sdf(vec3 p)
{
    SDFResult result;
    float s = 0.0;

    //// 1st object: plane
    float plane1H = -0.1;
    float distancePlane = sdfPlane(p, plane1H);
    vec3 colorPlane = vec3(1.0, 0.4, 0.4);

    //// 2nd object: sphere1
    vec3 sphere1C = vec3(-2.0, 1.0, 0.0);
    float sphere1R = 0.25;
    float distanceSphere1 = sdfSphere(p, sphere1C, sphere1R);
    vec3 colorSphere1 = vec3(0.0, 1.0, 0.0);

    //// 3rd object: box1
    vec3 box1C = vec3(-1.0, 1.0, 0.0);
    vec3 box1B = vec3(0.2, 0.2, 0.2);
    float distanceBox1 = sdfBox(p, box1C, box1B);
    vec3 colorBox1 = vec3(1.0, 0.0, 1.0);

    //// 4th object: box2 - sphere2
    vec3 box2C = vec3(0.0, 1.0, 0.0);
    vec3 box2B = vec3(0.3, 0.3, 0.3);
    vec3 sphere2C = vec3(0.0, 1.0, 0.0);
    float sphere2R = 0.4;
    float distanceBoxSubSphere = sdfSubtraction(sdfBox(p, box2C, box2B), sdfSphere(p, sphere2C, sphere2R));
    vec3 colorBoxSubSphere = vec3(1.0, 1.0, 0.0);

    //// 5th object: sphere3 intersect sphere4
    vec3 sphere3C = vec3(1.0, 1.0, 0.0);
    float sphere3R = 0.4;
    vec3 sphere4C = vec3(1.3, 1.0, 0.0);
    float sphere4R = 0.3;
    float distanceSphereIntersect = sdfIntersection(sdfSphere(p, sphere3C, sphere3R), sdfSphere(p, sphere4C, sphere4R));
    vec3 colorSphereIntersect = vec3(0.3, 0.7, 0.9);

    s = distancePlane;
    result.color = colorPlane;

    if(distanceSphere1 < s)
    {
        s = distanceSphere1;
        result.color = colorSphere1;
    }

    if(distanceBox1 < s)
    {
        s = distanceBox1;
        result.color = colorBox1;
    }

    if(distanceBoxSubSphere < s)
    {
        s = distanceBoxSubSphere;
        result.color = colorBoxSubSphere;
    }

    if(distanceSphereIntersect < s)
    {
        s = distanceSphereIntersect;
        result.color = colorSphereIntersect;
    }

    result.distance = s;
    return result;
}

/////////////////////////////////////////////////////
//// Step 7: creative expression
//// You will create your customized sdf scene with new primitives and CSG operations in the sdf2 function.
//// Call sdf2 in your ray marching function to render your customized scene.
/////////////////////////////////////////////////////

//smooth min using root
float smoothMin(float a, float b, float k) {
    k *= 2.0;
    float x = b- a;
    return 0.5*( a+b-sqrt(x*x+k*k) );
}

// smooth max using root
float smoothMax(float a, float b, float k) {
    k *= 2.0;
    float x = a - b;
    return 0.5 * (a + b + sqrt(x * x + k * k));
}

float smoothSdfIntersection(float s1, float s2, float k)
{

    return smoothMax(s1, s2, k);

}

float smoothSdfUnion(float s1, float s2, float k)
{
    
    return smoothMin(s1, s2, k);
}

float smoothSdfSubtraction(float s1, float s2, float k)
{
    
    return smoothMax(s1, -s2, k);
}

//Below SDFs are from https://iquilezles.org/articles/distfunctions/

//Capsule/Line SDF
float sdCapsule( vec3 p, vec3 a, vec3 b, float r )
{
  vec3 pa = p - a, ba = b - a;
  float h = clamp( dot(pa,ba)/dot(ba,ba), 0.0, 1.0 );
  return length( pa - ba*h ) - r;
}

float dot2( vec3 v ) { return dot(v,v); }

//Round Cone SDF
float sdRoundCone( vec3 p, vec3 a, vec3 b, float r1, float r2 )
{
  // sampling independent computations (only depend on shape)
  vec3  ba = b - a;
  float l2 = dot(ba,ba);
  float rr = r1 - r2;
  float a2 = l2 - rr*rr;
  float il2 = 1.0/l2;
    
  // sampling dependant computations
  vec3 pa = p - a;
  float y = dot(pa,ba);
  float z = y - l2;
  float x2 = dot2( pa*l2 - ba*y );
  float y2 = y*y*l2;
  float z2 = z*z*l2;

  // single square root!
  float k = sign(rr)*rr*rr*x2;
  if( sign(z)*a2*z2>k ) return  sqrt(x2 + z2)        *il2 - r2;
  if( sign(y)*a2*y2<k ) return  sqrt(x2 + y2)        *il2 - r1;
                        return (sqrt(x2*a2*il2)+y*rr)*il2 - r1;
}

//Cone SDF
float sdCone( vec3 p, vec2 c, float h )
{
  vec2 q = h*vec2(c.x/c.y,-1.0);
    
  vec2 w = vec2( length(p.xz), p.y );
  vec2 a = w - q*clamp( dot(w,q)/dot(q,q), 0.0, 1.0 );
  vec2 b = w - q*vec2( clamp( w.x/q.x, 0.0, 1.0 ), 1.0 );
  float k = sign( q.y );
  float d = min(dot( a, a ),dot(b, b));
  float s = max( k*(w.x*q.y-w.y*q.x),k*(w.y-q.y)  );
  return sqrt(d)*sign(s);
}

// sdf2: p - query point
SDFResult sdf2(vec3 p)
{
    SDFResult result;
    float distance = 0.0;

    //// Plane
    float planeHeight = -0.1;
    float distancePlane = sdfPlane(p, planeHeight);
    vec3 colorPlane = vec3(1.0, 0.4, 0.4);

    //// body (capsule + rounded cone)
    vec3 capsuleStart = vec3(-0.3, 0.5, 0.0);
    vec3 capsuleEnd = vec3(-0.3, 1.5, 0.0);
    float capsuleRadius = 0.3;
    float distanceCapsule = sdCapsule(p, capsuleStart, capsuleEnd, capsuleRadius);

    vec3 coneBase = capsuleStart;
    vec3 coneTip = vec3(-0.3, 1.0, 0.0);
    float coneR1 = 0.5;
    float coneR2 = 0.3;
    float distanceCone = sdRoundCone(p, coneBase, coneTip, coneR1, coneR2);

    float distanceBody = smoothSdfUnion(distanceCapsule, distanceCone, 0.2);
    vec3 colorBody = vec3(0.9, 0.9, 0.9);

    //// Head (Sphere)
    vec3 headCenter = vec3(-0.3, 1.8, 0.0);
    float headRadius = 0.45;
    float distanceHead = sdfSphere(p, headCenter, headRadius);

    float distancePenguin = smoothSdfUnion(distanceBody, distanceHead, 0.001);

    //// Arms (Left & Right Capsules)
    float armOffsetY = 0.6;
    float armOffsetZ = -0.2;

    vec3 leftArmStart = vec3(-0.3, 1.0 + armOffsetY, armOffsetZ);
    vec3 leftArmEnd = vec3(-1.1, 1.0, armOffsetZ);
    vec3 rightArmStart = vec3(-0.3, 1.0 + armOffsetY, armOffsetZ);
    vec3 rightArmEnd = vec3(0.5, 1.0, armOffsetZ);
    float armRadius = 0.1;

    float distanceLeftArm = sdCapsule(p, leftArmStart, leftArmEnd, armRadius);
    float distanceRightArm = sdCapsule(p, rightArmStart, rightArmEnd, armRadius);
    float distanceArms = smoothSdfUnion(distancePenguin, distanceLeftArm, 0.001);
    distanceArms = smoothSdfUnion(distanceArms, distanceRightArm, 0.001);

    //// Feet (boxes)
    float footOffsetX = 0.27;
    float footOffsetY = -0.5;
    float footOffsetZ = -0.3;
    vec3 leftFootCenter = vec3(-0.3 - footOffsetX, 0.5 + footOffsetY, footOffsetZ);
    vec3 rightFootCenter = vec3(-0.3 + footOffsetX, 0.5 + footOffsetY, footOffsetZ);
    vec3 footSize = vec3(0.2, 0.05, 0.2);
    vec3 colorOrange = vec3(1.0, 0.75, 0.0);

    float distanceLeftFoot = sdfBox(p, leftFootCenter, footSize);
    float distanceRightFoot = sdfBox(p, rightFootCenter, footSize);
    float distanceFeet = smoothSdfUnion(distanceArms, distanceLeftFoot, 0.035);
    distanceFeet = smoothSdfUnion(distanceFeet, distanceRightFoot, 0.035);

    //// Eyes (spheres)
    float eyeOffsetX = 0.15;
    float eyeOffsetY = 0.05;
    float eyeOffsetZ = -0.45;
    float eyeRadius = 0.05;
    vec3 leftEyeCenter = vec3(headCenter.x - eyeOffsetX, headCenter.y + eyeOffsetY, headCenter.z + eyeOffsetZ);
    vec3 rightEyeCenter = vec3(headCenter.x + eyeOffsetX, headCenter.y + eyeOffsetY, headCenter.z + eyeOffsetZ);

    float distanceLeftEye = sdfSphere(p, leftEyeCenter, eyeRadius);
    float distanceRightEye = sdfSphere(p, rightEyeCenter, eyeRadius);

    //// Beak (cone)
    float beakHeight = 0.15;
    vec2 beakC = vec2(0.577, 0.816); // 30-degree angle
    vec3 beakBase = vec3(headCenter.x, headCenter.y - 0.05, headCenter.z - 0.55);

    mat3 rotationMatrix = mat3(
        1.0,  0.0,  0.0,
        0.0,  0.0, -1.0,
        0.0, -1.0,  0.0
    );
    vec3 rotatedP = rotationMatrix * (p - beakBase);
    float distanceBeak = sdCone(rotatedP, beakC, beakHeight);
    float distanceFinal = distanceFeet;

    distance = distancePlane;
    vec3 finalColor = colorPlane;

    vec3 colorBlack = vec3(0.2, 0.2, 0.2);

    distanceFinal = distanceFeet;

    float epsilon = 0.05;

    if(distanceFinal < distance)
    {
        distance = distanceFinal;
        finalColor = colorBody;

        if (distanceFinal <= distanceLeftFoot + epsilon && distanceFinal >= distanceLeftFoot - epsilon) {
            finalColor = colorOrange;
        }
        else if (distanceFinal <= distanceRightFoot + epsilon && distanceFinal >= distanceRightFoot - epsilon) {
            finalColor = colorOrange;
        }
        else if (distanceFinal <= distanceLeftArm + epsilon && distanceFinal >= distanceLeftArm - epsilon) {
            finalColor = colorBlack;
        }
        else if (distanceFinal <= distanceRightArm + epsilon && distanceFinal >= distanceRightArm - epsilon) {
            finalColor = colorBlack;
        }
    }

    if (distanceBeak < distance) {
        distance = distanceBeak;
        finalColor = colorOrange;
    }

    if (distanceLeftEye < distance) {
        distance = distanceLeftEye;
        finalColor = colorBlack;
    }

    if (distanceRightEye < distance) {
        distance = distanceRightEye;
        finalColor = colorBlack;
    }


    result.color = finalColor;
    result.distance = distance;
    return result;
}



/////////////////////////////////////////////////////
//// ray marching
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//// Step 4: ray marching
//// You are asked to implement the ray marching algorithm within the following for-loop.
/////////////////////////////////////////////////////

//// ray marching: origin - ray origin; dir - ray direction 
float rayMarching(vec3 origin, vec3 dir)
{
    SDFResult result;
    result.distance = 0.0;
    result.color = vec3(1.0, 1.0, 1.0);

    float s = 0.0;
    for(int i = 0; i < 100; i++)
    {
        //// your implementation starts
        vec3 rayPoint = origin + (s * dir);
        float minDistanceToScene = sdf2(rayPoint).distance;
        if (minDistanceToScene <= 0.001) {
            return s;
        }
        s += minDistanceToScene;

        //// your implementation ends
    }
    
    return s;
}

/////////////////////////////////////////////////////
//// normal calculation
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//// Step 5: normal calculation
//// You are asked to calculate the sdf normal based on finite difference.
/////////////////////////////////////////////////////

//// normal: p - query point
vec3 normal(vec3 p)
{
    float s = sdf2(p).distance;          //// sdf value in p
    float dx = 0.01;           //// step size for finite difference

    //// your implementation starts
    float x_comp = s - sdf2(vec3(p.x - dx, p.y, p.z)).distance;
    float y_comp = s - sdf2(vec3(p.x, p.y - dx, p.z)).distance;
    float z_comp = s - sdf2(vec3(p.x, p.y, p.z - dx)).distance;
    
    return normalize(vec3(x_comp, y_comp, z_comp));

    //// your implementation ends
}

/////////////////////////////////////////////////////
//// Phong shading
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//// Step 6: lighting and coloring
//// You are asked to specify the color for each object in the scene.
//// Each object must have a separate color without mixing.
//// Notice that we have implemented the default Phong shading model for you.
/////////////////////////////////////////////////////

vec3 phong_shading(vec3 p, vec3 n)
{
    //// background
    if(p.z > 10.0){
        return vec3(0.9, 0.6, 0.2);
    }

    //// phong shading
    vec3 lightPos = vec3(4.*sin(iTime), 4., 4.*cos(iTime));  
    vec3 l = normalize(lightPos - p);               
    float amb = 0.1;
    float dif = max(dot(n, l), 0.) * 0.7;
    vec3 eye = CAM_POS;
    float spec = pow(max(dot(reflect(-l, n), normalize(eye - p)), 0.0), 128.0) * 0.9;

    vec3 sunDir = vec3(0, 1, -1);
    float sunDif = max(dot(n, sunDir), 0.) * 0.2;

    //// shadow
    float s = rayMarching(p + n * 0.02, l);
    if(s < length(lightPos - p)) dif *= .2;

    vec3 color = vec3(1.0, 1.0, 1.0);

    //// your implementation for coloring starts

    color = sdf2(p).color;

    //// your implementation for coloring ends

    return (amb + dif + spec + sunDif) * color;
}


/////////////////////////////////////////////////////
//// main function
/////////////////////////////////////////////////////

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    vec2 uv = (fragCoord.xy - .5 * iResolution.xy) / iResolution.y;         //// screen uv
    vec3 origin = CAM_POS;                                                  //// camera position 
    vec3 dir = normalize(vec3(uv.x, uv.y, 1));                              //// camera direction
    float s = rayMarching(origin, dir);                                     //// ray marching
    vec3 p = origin + dir * s;                                              //// ray-sdf intersection
    vec3 n = normal(p);                                                     //// sdf normal
    vec3 color = phong_shading(p, n);                                       //// phong shading
    fragColor = vec4(color, 1.);                                            //// fragment color
}

void main() 
{
    mainImage(gl_FragColor, gl_FragCoord.xy);
}