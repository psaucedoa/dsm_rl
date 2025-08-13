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
#define YAW_LEFT 5
#define YAW_RIGHT 6

#define BLADE_UP 7
#define BLADE_DOWN 8
#define CONTINUE 9

#define EMPTY 0
#define WALL 1
#define LAVA 2
#define GOAL 3
#define REWARD 4
#define OBJECT 5
#define AGENT_1 6

#define TIMESTEP 0.1

// ---------------------------------------------------------------

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
  float blade_yaw;
  float blade_height;

  float spawn_y;
  float spawn_x;
};

typedef struct Env Env;
struct Env
{
  float* observations;
  float* actions;
  float* rewards;
  float* dones;

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

  float max;
  double mean;
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
      env->height_map[adr] = 20;
    }
  }

  for (int c = 250; c < 300; c++)
  {
    for (int r = 250; r < 300; r++)
    {
      // sinusoidal
      int adr = grid_offset(env, r, c);
      int x = 0.1 * (c - 275);
      int y = 0.1 * (r - 275);

      env->height_map[adr] = -1*(x * x + y * y) + 30;
      // env->height_map[adr] = 50;
    }
  }

  // Agent spawning
  for (int i = 0; i < env->num_agents; i++)
  {
    Agent* agent = &env->agents[i];
    int adr = grid_offset(env, agent->spawn_y, agent->spawn_x);
    // assert(env->grid[adr] == EMPTY);
    agent->y = agent->spawn_y;
    agent->x = agent->spawn_x;
    agent->blade_width = 20;
    agent->blade_thick = 2;
    agent->blade_fore = 20;
    agent->theta = 110 * PI / 180;
  }
}

void gradient(Env* env)
{
  env->max = 0;
  env->mean = 0;

  for (int r = 1; r < env->height-1; r++)
  {
    for (int c = 1; c < env->width-1; c++)
    {
      int adr = grid_offset(env, r, c);
      int adr_x_r = grid_offset(env, r, c+1);
      int adr_y_d = grid_offset(env, r+1, c);

      env->dx_r[adr] = env->height_map[adr_x_r] - env->height_map[adr];
      env->dy_d[adr] = env->height_map[adr_y_d] - env->height_map[adr];

      env->mean += env->height_map[adr];

      if (env->height_map[adr] > env->max)
      {
        env->max = env->height_map[adr];
      }

    }
  }
  env->mean /= env->width * env->height;
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

  int n[20] = {-1, 0, -2, 1, -3, 2, -4, 3, -5, 4, -6, 5, -7, 6, -8, 7, -9, 8, -10, 9};

  for (int i = 0; i < agent->blade_width; i++)
  {
    int x_n = n[i];

    int direction;
    if (agent->vel == 0)
    {
      direction = 1;
    }
    else
    {
      direction = agent->vel / abs(agent->vel);
    }

    float theta = agent->theta;

    Vector2 cut_1, cut_2, deposit_1, deposit_2, deposit_3, yaw_1, yaw_2, yaw_3, yaw_4, yaw_5;

    yaw_1.x = x_n;
    yaw_2.x = x_n;
    yaw_3.x = x_n;
    yaw_4.x = x_n;
    yaw_5.x = x_n;

    yaw_1.y = 0;
    yaw_2.y = 1;
    yaw_3.y = 2;
    yaw_4.y = 3;
    yaw_5.y = 4;

    float blade_yaw = agent->blade_yaw;
    if (direction < 0)
    {
      blade_yaw *= -1;
    }

    yaw_1 = rotate(yaw_1, blade_yaw);
    yaw_2 = rotate(yaw_2, blade_yaw);
    yaw_3 = rotate(yaw_3, blade_yaw);
    yaw_4 = rotate(yaw_4, blade_yaw);
    yaw_5 = rotate(yaw_5, blade_yaw);

    cut_1.x = agent->x + (agent->blade_fore + yaw_1.y * direction) * sinf(1*theta) + (yaw_1.x) * cosf(1*theta);
    cut_1.y = agent->y + (agent->blade_fore + yaw_1.y * direction) * cosf(1*theta) - (yaw_1.x) * sinf(1*theta);
    cut_2.x = agent->x + (agent->blade_fore + yaw_2.y * direction) * sinf(1*theta) + (yaw_2.x) * cosf(1*theta);
    cut_2.y = agent->y + (agent->blade_fore + yaw_2.y * direction) * cosf(1*theta) - (yaw_2.x) * sinf(1*theta);

    deposit_1.x = agent->x + (agent->blade_fore + yaw_3.y * direction) * sinf(1*theta) + (yaw_3.x) * cosf(1*theta);
    deposit_1.y = agent->y + (agent->blade_fore + yaw_3.y * direction) * cosf(1*theta) - (yaw_3.x) * sinf(1*theta);
    deposit_2.x = agent->x + (agent->blade_fore + yaw_4.y * direction) * sinf(1*theta) + (yaw_4.x) * cosf(1*theta);
    deposit_2.y = agent->y + (agent->blade_fore + yaw_4.y * direction) * cosf(1*theta) - (yaw_4.x) * sinf(1*theta);
    deposit_3.x = agent->x + (agent->blade_fore + yaw_5.y * direction) * sinf(1*theta) + (yaw_5.x) * cosf(1*theta);
    deposit_3.y = agent->y + (agent->blade_fore + yaw_5.y * direction) * cosf(1*theta) - (yaw_5.x) * sinf(1*theta);

    int height_1 = env->height_map[grid_offset(env, cut_1.y, cut_1.x)];
    int height_2 = env->height_map[grid_offset(env, cut_2.y, cut_2.x)];

    if (true_blade_height <= height_1 || true_blade_height <= height_2)
    {
      float height_diff_1 = height_1 - true_blade_height;
      float height_diff_2 = height_2 - true_blade_height;

      float delta_soil = (height_diff_1 + height_diff_2) / 6;

      env->height_map[grid_offset(env, deposit_1.y, deposit_1.x)] += delta_soil * 2;
      env->height_map[grid_offset(env, deposit_2.y, deposit_2.x)] += delta_soil * 2;
      env->height_map[grid_offset(env, deposit_3.y, deposit_3.x)] += delta_soil * 2;
      env->height_map[grid_offset(env, cut_1.y, cut_1.x)] = true_blade_height;
      env->height_map[grid_offset(env, cut_2.y, cut_2.x)] = true_blade_height;
    }
    // printf("HIT! Shaved off %f\n", height - true_blade_height);
    printf("Blade interaction!\n\
    \tLocation:\t(%i, %i) \n\
    \tAgent Theta:\t%f \n\
    \tWith height:\t%i \n\
    \tBlade height:\t%f \n\
    \tAvg height:\t%f\n\
    \tEnv Max:\t%f\n\
    \tEnv Mean:\t%f\n",
    cut_1.x, cut_1.y, agent->theta, height_1, true_blade_height, agent->avg_height, env->max, env->mean);
  }

}

/**
 * Iterate!
 */
bool step(Env* env)
{
  bool done = false;
  for (int agent_idx = 0; agent_idx < env->num_agents; agent_idx++)
  {
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

    else if (action == YAW_LEFT)
    {
      Agent* agent = &env->agents[agent_idx];
      agent->blade_yaw += 0.01;

      if (agent->blade_yaw > 0.5)
      {
        agent->blade_yaw = 0.5;
      }
    }

    else if (action == YAW_RIGHT)
    {
      Agent* agent = &env->agents[agent_idx];
      agent->blade_yaw -= 0.01;

      if (agent->blade_yaw < -0.5)
      {
        agent->blade_yaw = -0.5;
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

    if (dest_y > env->height - 50)
    {
      dest_y = env->height - 50;
    }
    else if (dest_y < 50)
    {
      dest_y = 50;
    }
    else
    {
      agent->y = dest_y;
    }


    if (dest_x > env->width - 50)
    {
      dest_x = env->width - 50;
    }
    else if (dest_x < 50)
    {
      dest_x = 50;
    }
    else
    {
      agent->x = dest_x;
    }

    calculate_neighborhood_height(env);
    blade_interaction(env);

    int adr = grid_offset(env, y, x);
    int dest_adr = grid_offset(env, dest_y, dest_x);
    int dest_tile = env->grid[dest_adr];

    gradient(env);
    erode(env);
  }

  return done;
}

// Raylib client -------------------------------------------------------------//
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

Renderer* init_renderer(int cell_size, int width, int height)
{
  Renderer* renderer = (Renderer*)calloc(1, sizeof(Renderer));
  renderer->cell_size = cell_size;
  renderer->width = width;
  renderer->height = height;

  InitWindow(width*cell_size, height*cell_size, "PufferLib Ray Grid");
  SetTargetFPS(10);

  return renderer;
}

void close_renderer(Renderer* renderer)
{
  CloseWindow();
  free(renderer);
}

void render_debug(Renderer* renderer, Env* env)
{
  if (IsKeyDown(KEY_ESCAPE))
  {
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

void render_global(Renderer* renderer, Env* env)
{
  if (IsKeyDown(KEY_ESCAPE))
  {
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
      int height = env->height_map[adr] * 3;

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
  float deg = (agent->theta) * 180 / PI;

  Rectangle blade;
  blade.x  = agent->x * renderer->cell_size; // - agent->blade_width / 2;
  blade.y  = agent->y * renderer->cell_size; // - agent->blade_thick / 2;
  blade.height = agent->blade_thick * renderer->cell_size;
  blade.width  = agent->blade_width * renderer->cell_size;

  DrawRectanglePro(
  blade,
  (Vector2){10 * renderer->cell_size, -1*agent->blade_fore * renderer->cell_size},
  -1*deg,
  YELLOW
  );

  DrawCircle(
    agent->x * renderer->cell_size,
    agent->y * renderer->cell_size,
    5 * renderer->cell_size,
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
  return env;
}


void reset_room(Env* env)
{
  for (int r = 0; r < env->height; r++)
  {
    for (int c = 0; c < env->width; c++)
    {
      int adr = grid_offset(env, r, c);
      env->grid[adr] = EMPTY;
    }
  }
  reset(env, 0);
}
