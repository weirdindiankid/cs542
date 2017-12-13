from numpy import array
from keras.models import Sequential
from keras.layers import Dense
from matplotlib import pyplot
from keras import backend
 
def rmse(y_true, y_pred):
	return backend.sqrt(backend.mean(backend.square(y_pred - y_true), axis=-1))
 
# prepare sequence
X = array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
# create model
model = Sequential()
model.add(Dense(2, input_dim=1, activation='relu'))
model.add(Dense(1))
model.compile(loss='mse', optimizer='adam', metrics=[rmse])
# train model
history = model.fit(X, X, epochs=50000, batch_size=len(X), verbose=2)
# plot metrics
pyplot.plot(history.history['rmse'])
pyplot.show()
