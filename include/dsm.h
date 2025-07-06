#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "raylib.h"
#include "rlgl.h"

#define PASS 0
#define SPEED_UP 1
#define SPEED_DOWN 2
#define LEFT 3
#define RIGHT 4
#define BLADE_UP 5
#define BLADE_DOWN 6
#define CONTINUE 7

#define EMPTY 0
#define WALL 1
#define LAVA 2
#define GOAL 3
#define REWARD 4
#define OBJECT 5
#define AGENT_1 6
#define AGENT_2 7
#define AGENT_3 8
#define AGENT_4 9
#define AGENT_5 10
#define AGENT_6 11
#define AGENT_7 12
#define AGENT_8 13

#define NUM_AGENTS 8

#define TIMESTEP 0.1

// ---------------------------------------------------------------

int rand_color() {
    return AGENT_1 + rand()%(AGENT_4 - AGENT_1 + 1);
}

int is_agent(int tile) {
    return tile >= AGENT_1 && tile <= AGENT_8;
}

Vector2 rotate(Vector2 vector, float theta)
{
    Vector2 rotated = {
        vector.x * cosf(theta) + vector.y * sinf(theta),
        -1*vector.x * sinf(theta) + vector.y * cosf(theta)
    };
    return rotated;
}

typedef struct Agent Agent;
struct Agent
{
    float y;
    float x;
    float vel;
    float theta;  // gonna keep map aligned ot initial position, but following blade frame
    float theta_dot;
    float blade_pos;

    float avg_height;

    float blade_width;
    float blade_thick;
    int blade_fore;

    int accumulated_soil;
    int max_soil;

    float spawn_y;
    float spawn_x;

    int color;
};

typedef struct Env Env;
struct Env
{
    int width;
    int height;
    int num_agents;  // potential for multi-agent stuff
    int horizon;

    int cell_size;

    float meters_per_pixel;

    unsigned char* grid;
    float* height_map;
    float* dx_l;
    float* dx_r;
    float* dy_u;
    float* dy_d;
    Agent* agents;
    int action;
};

/**
 * Initialize grid values
 */
Env* init_grid(
    unsigned char* observations, unsigned int* actions, float* rewards, float* dones,
    int width, int height, int num_agents, int horizon,
    int vision, float speed, bool discretize) 
{
    Env* env = (Env*)calloc(1, sizeof(Env));

    env->width = width;
    env->height = height;
    env->num_agents = num_agents;
    env->horizon = horizon;

    env->grid = (unsigned char*)calloc(width*height, sizeof(unsigned char));
    env->height_map = (float*)calloc(width*height, sizeof(float));
    env->dx_l = (float*)calloc(width*height, sizeof(float));
    env->dx_r = (float*)calloc(width*height, sizeof(float));
    env->dy_u = (float*)calloc(width*height, sizeof(float));
    env->dy_d = (float*)calloc(width*height, sizeof(float));
    env->meters_per_pixel = 0.1;
    env->agents = (Agent*)calloc(num_agents, sizeof(Agent));
    return env;
}

/**
 * Allocate memory for env
 */
Env* allocate_grid(
    int width, int height, int num_agents, int horizon,
    int vision, float speed, bool discretize)
{
    int obs_size = 2*vision + 1;
    unsigned char* observations = (unsigned char*)calloc(num_agents*obs_size*obs_size, sizeof(unsigned char));
    unsigned int* actions = (unsigned int*)calloc(num_agents, sizeof(unsigned int));
    float* rewards = (float*)calloc(num_agents, sizeof(float));
    float* dones = (float*)calloc(num_agents, sizeof(float));
  
    return init_grid(observations, actions, rewards, dones,
        width, height, num_agents, horizon, vision, speed, discretize);
}

/**
 * Free env
 */
void free_env(Env* env)
{
    free(env->grid);
    free(env->height_map);
    free(env->agents);
    free(env);
}

/**
 * Free all allocated memory
 */
void free_allocated_grid(Env* env)
{
    free_env(env);
}

/**
 * Maps a 2d location to 1d array
 */
int grid_offset(Env* env, int y, int x)
{    
    return y*env->width + x;
}

int heightgrid_offset(Env* env, int y, int x)
{
    int y_scaled = y / env->cell_size;
    int x_scaled = x / env->cell_size;
    return y_scaled*env->width + x_scaled;
}

/**
 * Reset env
 */
void reset(Env* env, int seed)
{
    for (int r = 0; r < env->height; r++)
    {
        for (int c = 0; c < env->width; c++)
        {
            int adr = grid_offset(env, r, c);
            env->height_map[adr] = 120;
        }
    }

    for (int c = 0; c < env->width; c++)
    {
        for (int r = 0; r < env->height; r++)
        {
            // sinusoidal
            int adr = grid_offset(env, r, c);
            env->height_map[adr] = 20 * cosf(0.07*r - 200) + 140;
            // env->height_map[adr] = 50;
        }
    }

    // Agent spawning
    for (int i = 0; i < env->num_agents; i++)
    {
        Agent* agent = &env->agents[i];
        int adr = grid_offset(env, agent->spawn_y, agent->spawn_x);
        // assert(env->grid[adr] == EMPTY);
        // assert(is_agent(agent->color));
        agent->y = agent->spawn_y;
        agent->x = agent->spawn_x;
        agent->blade_width = 20;
        agent->blade_thick = 2;
        agent->blade_fore = 20;
        agent->max_soil = 10000;
        // env->grid[adr] = agent->color;
        agent->theta = 0;
    }
}

void gradient(Env* env)
{
    for (int r = 1; r < env->height-1; r++)
    {
        for (int c = 1; c < env->width-1; c++)
        {
            int adr = grid_offset(env, r, c);
            // int adr_x_l = grid_offset(env, r, c-1);
            int adr_x_r = grid_offset(env, r, c+1);
            // int adr_y_u = grid_offset(env, r-1, c);
            int adr_y_d = grid_offset(env, r+1, c);

            // env->dx_l[adr] = env->height_map[adr_x_l] - env->height_map[adr];
            env->dx_r[adr] = env->height_map[adr_x_r] - env->height_map[adr];
            // env->dy_u[adr] = env->height_map[adr_y_u] - env->height_map[adr];
            env->dy_d[adr] = env->height_map[adr_y_d] - env->height_map[adr];
        }
    }
}

void erode(Env *env)
{
    for (int r = 1; r < env->height-2; r++)
    {
        for (int c = 1; c < env->width-2; c++)
        {
            int adr = grid_offset(env, r, c);
            int adr_dx_l = grid_offset(env, r, c-1);
            int adr_dy_u = grid_offset(env, r-1, c);

            float dx_r = env->dx_r[adr];
            float dx_l = -1*env->dx_r[adr_dx_l];
            float dy_d = env->dy_d[adr];
            float dy_u = -1*env->dy_d[adr_dy_u];

            int adr_x_l = grid_offset(env, r, c-1);
            int adr_x_r = grid_offset(env, r, c+1);
            int adr_y_u = grid_offset(env, r-1, c);
            int adr_y_d = grid_offset(env, r+1, c);

            float grads[4] = {dx_l, dx_r, dy_u, dy_d};
            int adrs[4] = {adr_x_l, adr_x_r, adr_y_u, adr_y_d};

            int index = 0;
            float min = 0;

            for (int i = 0; i < 4; i++)
            {
                if (grads[i] < min)
                {
                    min = grads[i];
                    index = i;
                }
            }

            if (min < -2)
            {
                float diff = 0.5 * grads[index];
                env->height_map[adr] += diff;
                env->height_map[adrs[index]] -= diff;
            }
        }
    }
}

/**
 * Plane fitting thing
 */
void calculate_neighborhood_height(Env* env)
{
    // this should just calculate the best-fit plane. 
    // In this case, can't we just do average height?
    // Anyway, used to set relative blade height
    float x = env->agents[0].x;
    float y = env->agents[0].y;

    int neighborhood_len = 100;
    int neighborhood_width = 50;

    Vector2 top_left = {x,y};

    int sum = 0;
    int count = 0;

    for (int x_n = 0; x_n < neighborhood_width; x_n++)
    {
        for (int y_n = 0; y_n < neighborhood_len; y_n++)
        {
            Vector2 current_pixel = {x_n - neighborhood_width / 2, y_n - neighborhood_len / 2};
            Vector2 current_pixel_rotated = rotate(current_pixel, env->agents[0].theta);
            Vector2 current_coord = {top_left.x + current_pixel_rotated.x, top_left.y + current_pixel_rotated.y };

            sum += env->height_map[grid_offset(env, current_coord.y, current_coord.x)];

            // env->height_map[grid_offset(env, current_coord.y, current_coord.x)] = 40;

            count += 1;
        }
    }

    env->agents[0].avg_height = sum / count;
    // env->agents[0].avg_height = 10;
    // printf("Avg height: %f\n", env->agents[0].avg_height);
}

void blade_interaction(Env* env)
{
    Agent* agent = &env->agents[0];
    float true_blade_height = agent->avg_height + agent->blade_pos;
    int overflow;

    for (int x_n = 0; x_n < agent->blade_width; x_n++)
    {
        int direction;
        if (agent->vel == 0)
        {
            direction = 0;
        }
        else
        {
            direction = agent->vel / abs(agent->vel);
        }

        int x = agent->x + agent->blade_fore * sinf(1*agent->theta) + (x_n - agent->blade_width / 2) * cosf(1*agent->theta);
        int y_1 = agent->y + agent->blade_fore * cosf(1*agent->theta) - (x_n - agent->blade_width / 2) * sinf(1*agent->theta);
        int y_2 = agent->y + (agent->blade_fore + 1*direction) * cosf(1*agent->theta) - (x_n - agent->blade_width / 2) * sinf(1*agent->theta);

        int y_deposit_1 = agent->y + (agent->blade_fore + 3 * direction) * cosf(1*agent->theta) - (x_n - agent->blade_width / 2) * sinf(1*agent->theta);
        int y_deposit_2 = agent->y + (agent->blade_fore + 4 * direction) * cosf(1*agent->theta) - (x_n - agent->blade_width / 2) * sinf(1*agent->theta);
        int y_deposit_3 = agent->y + (agent->blade_fore + 5 * direction) * cosf(1*agent->theta) - (x_n - agent->blade_width / 2) * sinf(1*agent->theta);


        int height_1 = env->height_map[grid_offset(env, y_1, x)];
        int height_2 = env->height_map[grid_offset(env, y_2, x)];

        if (true_blade_height <= height_1 || true_blade_height <= height_2)
        {
            float height_diff_1 = height_1 - true_blade_height;
            float height_diff_2 = height_2 - true_blade_height;

            float delta_soil = (height_diff_1 + height_diff_2)/3;

            env->height_map[grid_offset(env, y_deposit_1, x)] += delta_soil;
            env->height_map[grid_offset(env, y_deposit_2, x)] += delta_soil;
            env->height_map[grid_offset(env, y_deposit_3, x)] += delta_soil;
            env->height_map[grid_offset(env, y_1, x)] = true_blade_height;
            env->height_map[grid_offset(env, y_2, x)] = true_blade_height;
        }
        // printf("HIT! Shaved off %f\n", height - true_blade_height);
        printf("Blade interaction!\n\
            \tLocation:\t(%i, %i) \n\
            \tWith height:\t%i \n\
            \tBlade height:\t%f \n\
            \tAvg height:\t%f\n\
            \tAcc soil:\t%i\n",
            x, y_1, height_1, true_blade_height, agent->avg_height, agent->accumulated_soil);
    }

}

/**
 * Iterate!
 */
bool step(Env* env)
{
    // TODO: Handle discrete vs continuous
    /*
    if self.discretize:
        actions_discrete = np_actions
    else:
        actions_continuous = np_actions
    */
    bool done = false;
    for (int agent_idx = 0; agent_idx < env->num_agents; agent_idx++)
    {
        // Discrete case only
        // int atn = env->actions[agent_idx];
        int action = env->action;
        float vel = 0;

        if (action == PASS)
        {
            continue;
        }
        else if (action == CONTINUE)
        {
            // continue;
        }

        else if (action == SPEED_UP)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->vel += 1;

            if (agent->vel > 10)
            {
                agent->vel = 10;
            }
        }
        
        else if (action == SPEED_DOWN)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->vel -= 1;
        
            if (agent->vel < -10)
            {
                agent->vel = -10;
            }
        }
        
        else if (action == LEFT)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->theta_dot += 0.1;
            if (agent->theta_dot > 1)
            {
                agent->theta_dot = 1;
            }
        }

        else if (action == RIGHT)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->theta_dot -= 0.1;

            if (agent->theta_dot < -1)
            {
                agent->theta_dot = -1;
            }
        }

        else if (action == BLADE_UP)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->blade_pos += 1;

            if (agent->blade_pos > 15)
            {
                agent->blade_pos = 15;
            }
        }

        else if (action == BLADE_DOWN)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->blade_pos -= 1;

            if (agent->blade_pos < -10)
            {
                agent->blade_pos = -10;
            }

        }

        else
        {
            printf("Invalid action: %i\n", action);
            exit(1);
        }


        Agent* agent = &env->agents[agent_idx];
        agent->theta += agent->theta_dot * TIMESTEP;
        if (agent->theta > 2*PI)
        {
            agent->theta -= 2*PI;
        }
        if (agent->theta < 0)
        {
            agent->theta += 2*PI;
        }

        float y = agent->y;
        float x = agent->x;

        // we have the speed and heading (theta)
        // 0 is straight up, 90 is left...
        // also, this might be costly?

        float dest_y = TIMESTEP * agent->vel * cosf(agent->theta) + y;
        float dest_x = TIMESTEP * agent->vel * sinf(agent->theta) + x;

        agent->y = dest_y;
        agent->x = dest_x;

        calculate_neighborhood_height(env);
        blade_interaction(env);

        int adr = grid_offset(env, y, x);
        int dest_adr = grid_offset(env, dest_y, dest_x);
        int dest_tile = env->grid[dest_adr];

        // for (int i = 0; i < 4; i++)
        // {
            gradient(env);
            erode(env);
            // i++;
        // }
    }

    return done;
}

// Raylib client
Color COLORS[] = {
    (Color){6, 24, 24, 255},
    (Color){0, 0, 255, 255},
    (Color){0, 128, 255, 255},
    (Color){128, 128, 128, 255},
    (Color){255, 0, 0, 255},
    (Color){255, 255, 255, 255},
    (Color){255, 85, 85, 255},
    (Color){170, 170, 170, 255},
    (Color){0, 255, 255, 255},
    (Color){255, 255, 0, 255},
};

Rectangle UV_COORDS[7] = {
    (Rectangle){0, 0, 0, 0},
    (Rectangle){512, 0, 128, 128},
    (Rectangle){0, 0, 0, 0},
    (Rectangle){0, 0, 128, 128},
    (Rectangle){128, 0, 128, 128},
    (Rectangle){256, 0, 128, 128},
    (Rectangle){384, 0, 128, 128},
};

typedef struct {
    int cell_size;
    int width;
    int height;
    Texture2D gato;
} Renderer;

Renderer* init_renderer(int cell_size, int width, int height) {
    Renderer* renderer = (Renderer*)calloc(1, sizeof(Renderer));
    renderer->cell_size = cell_size;
    renderer->width = width;
    renderer->height = height;

    InitWindow(width*cell_size, height*cell_size, "PufferLib Ray Grid");
    SetTargetFPS(100);

    return renderer;
}

void close_renderer(Renderer* renderer) {
    CloseWindow();
    free(renderer);
}

void render_debug(Renderer* renderer, Env* env)
{
    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }
    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});

    int ts = renderer->cell_size;
    for (int r = 0; r < env->height; r++)
    {
        for (int c = 0; c < env->width; c++)
        {
            int adr = grid_offset(env, r, c);
            int dx = env->dx_r[adr];
            dx *= 5;
            dx += 100;
            DrawRectangle(c*ts, r*ts, ts, ts, (Color){dx, dx, dx, 255});

        }
    }
    EndDrawing();
}

void render_global(Renderer* renderer, Env* env) {
    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});

    int ts = renderer->cell_size;
    for (int r = 0; r < env->height; r++)
    {
        for (int c = 0; c < env->width; c++)
        {
            int adr = grid_offset(env, r, c);
            int height = env->height_map[adr];

            if (height > 255)
            {
                DrawRectangle(c*ts, r*ts, ts, ts, (Color){255, 0, 0, 255});
            }
            else
            {
                DrawRectangle(c*ts, r*ts, ts, ts, (Color){height, height, height, 255});
            }

        }
    }

    Agent* agent = &env->agents[0];
    float deg = agent->theta * 180 / PI;

    Rectangle blade;
    blade.x = agent->x; // - agent->blade_width / 2;
    blade.y = agent->y; // - agent->blade_thick / 2;
    blade.height = agent->blade_thick;
    blade.width = agent->blade_width;

    // draw blade
    // DrawRectangle(agent->x, agent->y, agent->blade_width, agent->blade_thick, (Color){255, 255, 255, 255});
    DrawRectanglePro(
        blade,
        (Vector2){10, -1*agent->blade_fore},
        -1*deg,
        YELLOW
    );

    DrawCircle(
        agent->x,
        agent->y,
        5,
        RED
    );

    DrawText(TextFormat("A-VEL: %02.02f deg/s", agent->theta_dot), 20, 20, 10, WHITE);
    DrawText(TextFormat("THETA: %02.02f deg", deg), 20, 40, 10, WHITE);
    DrawText(TextFormat("SPEED: %02.02f px/s", agent->vel), 20, 60, 10, WHITE);
    DrawText(TextFormat("X POS: %02.02f px", agent->x), 20, 80, 10, WHITE);
    DrawText(TextFormat("Y POS: %02.02f px", agent->y), 20, 100, 10, WHITE);
    DrawText(TextFormat("BLADE: %02.02f px", agent->blade_pos), 20, 120, 10, WHITE);
    EndDrawing();

}

Env* alloc_room_env()
{
    int width = 500;
    int height = 500;
    int num_agents = 1;
    int horizon = 512;
    float agent_speed = 1;
    int vision = 3;
    bool discretize = true;

    Env* env = allocate_grid(width+2*vision, height+2*vision, num_agents, horizon,
            vision, agent_speed, discretize);
    
    env->cell_size = 1;
    env->agents[0].spawn_y = 150;
    env->agents[0].spawn_x = 150;
    env->agents[0].color = AGENT_1;
    return env;
}


void reset_room(Env* env) {
    for (int r = 0; r < env->height; r++) {
        for (int c = 0; c < env->width; c++) {
            int adr = grid_offset(env, r, c);
            env->grid[adr] = EMPTY;
        }
    }
    reset(env, 0);
}
