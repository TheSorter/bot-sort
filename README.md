# bot-sort

Copied from https://devpost.com/software/bot-sort-aka-kwik-sort

Won 3rd place best project at HackTheU 2016

Inspiration:
Many materials like plastics and metals, which could be reused, end up wasting away in landfills. This is a large problem. But recycling is a pain. Getting people to spend the time to separate their waste in unrealistic. We thought: why not use the latest technology in machine learning and robotics to do this task for us? Using this technology would allow for the expanded use of recycling in our community. Consumers now do not have to worry about separating out what is recyclable and what is not, as this is taken care of by Bot Sort when the waste is processed. This has the overall implication that the world as a whole will become healthier and better because of reduced landfill use.

What it does:
Bot Sort takes a picture of a particular item to be sorted. Using machine learning and neural net algorithms, it determines what categories, such as aluminum, plastic, or trash, the item falls into. This information is then sent to a robotic arm which pushes the item into the proper bin. There is also web dashboard to track categories of objects that the system encounters.

How we built it:
Bot Sort is split into two main components, the machine learning object recognition and the physical robotic arm. The computer vision/machine learning portion is based on TensorFlow and written in python. We use a image training data set to train a neural network to determine what is featured in a photo. A snap shot of the item to recognize is then fed into this network where it takes it's best guess as to what the item is.

The robotic arm is controlled by an arduino which receives commands from the computer. These commands tell the arm to move in a specific pattern that will knock the object into one of the categorization bins. The arduino is running a program written in C that interprets the incoming serial commands and performs the desired action.

Challenges we ran into:
Library and camera support were some major challenges with this project. It took some time to figure out which libraries supported which version of python or vice versa. Learning the libraries also took tome time. Since our group was using a combination of 3 different operating systems, it was a challenge to figure out to access the camera on each system. We wanted to create some type of web interface to track the objects that the robot classified, and initially planned on using a javascript library, but we realized we didn't have time to implement. As a result, we ended up Microsofts power BI platform to create a real-time chart to track the object categories.

Accomplishments that we're proud of:
We are proud that we were able to meld the two world of hardware and software into one product that performs the task we designed it for.

What we learned:
We learned how to implement a neural network in tensor flow and also learned how to use some of the features available on Microsoft's azure.

What's next for Bot Sort AKA Kwik Sort:
The next step for Bot Sort/Kwik Sort is retraining the machine learning algorithm to better recognize the specific materials we will be sorting. We are also looking forward to upgrading our arm to have more power when moving the items.

Built With:
python
tensorflow
machine-learning
robotics
arduino
computer-vision
