# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 13:13:21 2023

@author: anton
"""

#%% Imports
import numpy as np
import gymnasium as gym
import matplotlib.pyplot as plt
import pygmo as pg
import torch
from torch import nn, tensor
from skorch import NeuralNetRegressor
from skorch import NeuralNetBinaryClassifier
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GridSearchCV, train_test_split
import time

#https://gymnasium.farama.org/environments/classic_control/pendulum/

#%% Initializing
start_time = time.time()
main_env = gym.make('Pendulum-v1',g=9.81)

horizon = 5 #horizon = 5, gen=3 and population; size = 10 leads to OK results at decent computations times 

#%% Defining cost function
def cost_function(params, action, horizon):    
    env_mpc = gym.make('Pendulum-v1', g=9.81)
    state,_ = env_mpc.reset()
    env_mpc.unwrapped.state = params #main_env.unwrapped.state
    
    weight_theta = 0.1
    weight_omega = 0.1  
    weight_action = 0.001  
    theta_setpoint = 0
    omega_setpoint = 0  
    cost = 0
    for act in action: #runs 10 times because horizon=10
        obs, _, _, _,_ = env_mpc.step([act])
        cos_theta, sin_theta, omega = obs        #The observation space is x, y and omega
        theta = np.arctan2(sin_theta, cos_theta) #theta is needed, not x or y
        cost += weight_theta * ((theta - theta_setpoint)**2 + weight_omega * (omega - omega_setpoint) ** 2 + weight_action * act ** 2)
    return cost

#%% Defining the tuning class
class MPC_tuning:
    def __init__(self,params,horizon):
        self.params = params
        self.horizon = horizon
        
    def fitness(self, action):
        cost = cost_function(self.params, action, self.horizon) #this cost is computed 500 times at 30x30 gen pop, and 100 times if 10x10 gen pop
        return [cost]

    def get_bounds(self):
        return ([-2.]*self.horizon, [2.]*self.horizon) #Defines the bounds of the action [Torque]

#%% Play game
def play_game(initial_cond, env, controller, printing=False, model=None):
    terminated = False
    truncated = False
    state, _ = env.reset()
    env.unwrapped.state = initial_cond  #Initializing the state
    
    observations = []
    controls = []
    total_score = 0
    counter = 0  
    prev_obs = initial_cond
    horizon = 5 #unnecessary
    
    while not (terminated or truncated):
        if controller == "RANDOM":
            action = env.action_space.sample()[0]
        
        elif controller == "MPC":
            prob = pg.problem(MPC_tuning(prev_obs, horizon))
            algo = pg.algorithm(pg.sga(gen=10)) #Simple genetic algorithm
            pop = pg.population(prob, size=10)
            pop = algo.evolve(pop)           
            best_controls = pop.champion_x     #Finding best controls
            action = best_controls[0]
            
        elif controller == "NN":
                prev_obs_NN = torch.tensor([prev_obs[0],prev_obs[1],prev_obs[2]],dtype=torch.float32)
                prev_obs_NN = prev_obs_NN.reshape(1,-1)
                action = model.predict(prev_obs_NN).item()

        controls.append(action)
        obs, reward, terminated, truncated, _ = env.step([action])

        observations.append(obs)
        prev_obs = obs #Need it to be a [3,] array not a [3,1] for prev_obs to be the correct shape for action = model.predict(prev_obs).item()
        total_score += reward
        
        if printing == True: #If desired the duration of the interation can be followed
            counter += 1
            if counter % 100 == 0:
                print(f'Iteration: {counter}/200 max')
    return observations, controls, total_score
#%% Play_X_games - This function can be used to play multiple games
def play_X_games(num_games, controller, env, printing = False, model = None):
    scores = []
    obs = []
    action = []
    if controller == 'MPC':
        print('Controller: MPC')
    elif controller == 'RANDOM':
        print('Controller: RANDOM')
    elif controller =='NN':
        print('Controller: NN')
    for i in range(num_games):
        angle = np.random.uniform(-20*np.pi/180, 20*np.pi/180)
        initial_cond = [np.cos(angle), np.sin(angle), np.random.uniform(-0.1, 0.1)]
        if printing == True:
            print(f'Game no. {i+1}/{num_games} ')
        observations, controls, total_score = play_game(initial_cond, env, controller, printing, model) #Calling the play_game function
        scores.append(total_score)
        obs.append(observations)
        action.append(controls)
        if printing == True:
            print()
    if printing == True:
        print(f'Scores from {num_games} games: \n{np.around(np.array(scores), decimals=4)} \n')
    print(f'Average Score [{controller}]: {np.sum(scores)/num_games:.4f}\n')
    return obs, action, scores

#%% Generate_training_data. Should generate 2000 sets of observations+action
def generate_training_data(num_games, env, printing = False):
    training_data = []
    counter = 0
    for i in range(num_games):
        score = 0
        angle = np.random.uniform(-20*np.pi/180, 20*np.pi/180)
        initial_cond = [np.cos(angle), np.sin(angle), np.random.uniform(-0.1, 0.1)]
        state, _ = env.reset()
        env.unwrapped.state = initial_cond
        game_memory = []
        prev_obs = initial_cond

        terminated = False
        truncated = False
        while not (terminated or truncated):
            prob = pg.problem(MPC_tuning(prev_obs, horizon))
            algo = pg.algorithm(pg.sga(gen=5)) #Simple genetic algorithm
            pop = pg.population(prob, size=10)
            pop = algo.evolve(pop)           
            best_controls = pop.champion_x     #Finding best controls
            action = best_controls[0]
            
            obs, reward, terminated, truncated, _ = env.step([action])
            
            if len(prev_obs) > 0:
                game_memory.append([prev_obs, action])
            
            prev_obs = obs
            score += reward                       
            
        if score > -2: #If the score of a game is sufficiently good - store it
            for data in game_memory:
                observations = data[0]
                actions = data[1]
                training_data.append([observations, actions])  
        if printing == True:
            counter += 1
            print(f'Datasets generated: {counter*200}')
            print(f"Score: {score:.4f}")    
    return training_data
    
#%% Plotting
def plot_res(observations, controls):
    observations = np.array(observations)
    plt.figure(figsize=(10, 5))
    for i in range(len(observations)):
        theta = np.arctan2(observations[i,:,1], observations[i,:,0])
        angular_velocities = observations[i,:,2]
        plt.subplot(3, 1, 1)
        plt.plot(theta)
        plt.title('Angles')
        plt.xlabel('Steps')
        plt.ylim([-np.pi, np.pi])
        plt.ylabel('Angle')
    
        plt.subplot(3, 1, 2)
        plt.plot(angular_velocities)
        plt.title('Angular Velocities')
        plt.xlabel('Steps')
        plt.ylim([-8, 8])
        plt.ylabel('Angular Velocity')
    
        plt.subplot(3, 1, 3)
        plt.plot(controls[i])
        plt.title('Controls')
        plt.xlabel('Steps')
        plt.ylabel('Control')
    
    plt.tight_layout()
    plt.show()

def rendering(initial_cond, controls):
   # Initialize the environment
   env = gym.make('Pendulum-v1', g=9.81, render_mode="human")
   state, _ = env.reset()
   env.unwrapped.state = initial_cond
   observations = []
   terminated = False
   truncated = False
   # Step through the environment using your controls
   while not (terminated or truncated):
       for control in controls:
           obs, reward, terminated,truncated, info = env.step([control])
           observations.append(obs)
       env.close()
   if truncated:
       print('\nRendering truncated \n')
       
#%% Score of X games

num_games = 2
#observations_rand, controls_rand, total_scores_rand = play_X_games(num_games, 'RANDOM', main_env)

#observations_mpc, controls_mpc, total_scores_mpc = play_X_games(num_games, 'MPC', main_env, printing = True)
elapsed_time = time.time() - start_time
print(f"\nElapsed time for Part 1: {elapsed_time:.4f} seconds")

#%%
#plot_res([observations_mpc[0]],[controls_mpc[0]])

#%%
#angle = np.random.uniform(-20*np.pi/180, 20*np.pi/180)
#initial_cond = [np.cos(angle),np.sin(angle), np.random.uniform(-0.1, 0.1)]
#observations, controls, total_score = play_game(initial_cond, main_env, 'MPC')
#rendering(initial_cond, controls)

#%%
start_time = time.time()
#training_data = generate_training_data(2, main_env, printing = True)
elapsed_time = time.time() - start_time
print(f"\nElapsed time for generating training data: {elapsed_time:.4f} seconds")

#%% Part 2: Neural Network
# Initializing
data = np.load(r'C:/Users/anton/OneDrive - Aarhus Universitet/7. semester/Data Science/Final Assignment/training_pendulum.npy')
data_2000 = data[4000:6000,:]
X = data_2000[:,0:3]
y = data_2000[:,3].reshape(-1,1)
# %% Splitting
#X = np.array([entry[0] for entry in training_data[:2000]])
#y = np.array([entry[1] for entry in training_data[:2000]]).reshape(-1,1)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)

# %% NN
X_ = torch.tensor(X_train, dtype=torch.float32)
y_ = torch.tensor(y_train, dtype=torch.float32)

#%% Defining the neural network
class NN(nn.Module): #Simple network with only 1 hidden layer
    def __init__(self,num_units=32):
        super().__init__()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(3, num_units),
            nn.ReLU(),
            nn.Linear(num_units,1),
            #nn.Tanh()            
        )
    def forward(self, x):
        x = x.to(torch.float32)
        y = self.linear_relu_stack(x)
        #y = 2*self.linear_relu_stack(x)
        y = torch.clip(y,-2,2) #To make sure the actionbounds goes from -2 to 2
        return y 

#%% Defining Model
start_time = time.time()

epochs = 50
lr = 0.01

from skorch.callbacks import EarlyStopping #This stops the epochs if validation starts to go up

model = NeuralNetRegressor(module=NN, lr=lr, verbose=0, max_epochs=epochs, module__num_units=32, 
                           batch_size=64, callbacks=[('estoper',EarlyStopping(patience=30)),],
                           iterator_train__shuffle = True, optimizer__weight_decay=0.,optimizer=torch.optim.Adam, criterion=torch.nn.MSELoss,)

#%% Pipeline
pipe = Pipeline([
    ('scale',StandardScaler()),
    ('model',model),
])
#%% Pipe fit
pipe.fit(X_, y_) #Fitting model using the parameters that was specified in class NN

#%% Plotting training loss from pipe
train_loss = pipe['model'].history[:, 'train_loss']
valid_loss = pipe['model'].history[:, 'valid_loss']
epochs = [i for i in range(1,len(train_loss)+1)]
plt.figure()
plt.plot(epochs, train_loss, label='training loss')
plt.plot(epochs, valid_loss, label='validation loss')
plt.title('Pipe: Losses')
plt.legend()
plt.show()

#%% Pipe score
pipe.score(X_, y_)

#%% Gridsearch
params = {
    #"model__module__num_units": [10, 20, 40, 60],
    #"model__lr": [0.0001, 0.001, 0.01],
    "model__module__num_units": [40],
    "model__lr": [0.001],
}
gs = GridSearchCV(pipe,params,verbose=0) #Performing grid search to determine the best parameters of num_units and learning rate
gs.fit(X_, y_) #Pipe fit fits the model I have in my class NN, which is not the best model. gs is better (according to Alessandro) and I think it is because you tell it to try different learning rates, no. hidden units etc.
print("\ngs.best_score_: %0.4f:" % gs.best_score_)
print(gs.best_params_)
print()

model_info = gs.best_estimator_['model'].history
train_loss = model_info[:,'train_loss']
valid_loss = model_info[:,'valid_loss']
plt.figure()
plt.plot(train_loss, label = 'Training Loss')
plt.plot(valid_loss, label = 'Validations Loss')
plt.title('gs: Losses')
plt.legend()

#%% Playing 2000 games
num_games = 10 #Takes about 5 minutes for 2000 games and for only 1 set of params
obs_rand, controls_rand, scores_rand = play_X_games(num_games, 'RANDOM', main_env)
obs_NN, controls_NN, scores_NN = play_X_games(num_games, 'NN', main_env, printing=False, model=gs)

plt.figure()
plt.plot(scores_NN, label = 'Neural Network')
plt.plot(scores_rand, label = 'Random')
plt.xlabel('Game')
plt.ylabel('Score')
plt.title('Comparison of scores for NN and random controller')
plt.legend(loc='center right')

#Plotting 5 last games on top
plot_res(obs_NN[-5:],controls_NN[-5:])

elapsed_time = time.time() - start_time
print(f"\nElapsed time for Part 2: {elapsed_time:.4f} seconds")

#%% Rendering
angle = np.random.uniform(-20*np.pi/180, 20*np.pi/180)
initial_cond = [np.cos(angle),np.sin(angle), np.random.uniform(-0.1, 0.1)]
observations, controls, total_score = play_game(initial_cond, main_env, 'NN', model=gs.best_estimator_)
rendering(initial_cond, controls)



