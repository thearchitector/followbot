#ifndef structs_h
#define structs_h

// TODO: Verify
typedef struct vec3i {
    union {
        int vals [3];
        struct {
            int xR;
            int yR;
            int thetaR;
        };
        struct {
            int x;
            int y;
            int z;
        }
    };
} vec3i;

#endif
