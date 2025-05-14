import os
import pickle

from skimage.io import imread
from skimage.transform import resize
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.model_selection import GridSearchCV
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score


# prepare data
# Datenbank kann bei Lennart Treunert angefragt werden (5k Bilder)
input_dir = '-.-'  # Path to the directory containing the images has to be changed
categories = ['left', 'right', 'straight', 'crosswalk', 'park']

data = []
labels = []

for category_id, category in enumerate(categories):
    for files in os.listdir(os.path.join(input_dir, category)):
        img_path = os.path.join(input_dir, category, files)
        img = imread(img_path)
        img = resize(img, (img.shape[0]//4, img.shape[1]//4))
        data.append(img.flatten())
        labels.append(category_id)

data = np.asarray(data)
labels = np.asarray(labels)

# which label stands for which category?
# Print the mapping of labels to categories
print("Label mapping:")
for i, category in enumerate(categories):
    print(f"Label {i}: {category}")
# train / test split

# Split the data into training and testing sets
# 80% for training and 20% for testing
# stratify = labels ensures that the split maintains the same proportion of classes in both sets
x_train, x_test, y_train, y_test = train_test_split(data, labels, test_size=0.2, shuffle=True, stratify=labels)

# train classifier

classifier = SVC()
# list of two keys containing the hyperparameters to be tuned
parameters = [{'gamma': [0.01, 0.001], 'C': [1, 10]}]

grid_search = GridSearchCV(classifier, parameters, verbose=2)
print("Fitting the model...")
grid_search.fit(x_train, y_train)


# test performance
print("start searching best estimator...")
best_estimator = grid_search.best_estimator_

y_prediction = best_estimator.predict(x_test)
print("accuracy testing...")
score = accuracy_score(y_prediction, y_test)
print(f"Accuracy: {score * 100:.2f}%")


# save model
pickle.dump(best_estimator, open('sign_model.joblib', 'wb'))
