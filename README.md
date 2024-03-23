
## Setup

To start, first thing you should do is to clone the repository. To do this, open a directory where you want the project to be cloned to then run the following in command prompt:
```bash
git clone https://github.com/JohnEdrickGador/nec-mesh.git
```
After running this code, a new folder called "nec-mesh" should appear. Run the following to go to that folder and open the code in your text editor:
```bash
cd nec-mesh
code .
```
The next thing that we have to do is fetch all the remote branches so that we can have access to it. To do this, run this command:
```bash
git fetch
```
After running the git fetch command, we can now switch between the branches that are present and available on our github repository by doing the following command:
```bash
git checkout <branch-name>
```
Example:
```bash
git checkout hivemq
```
What this does is it serves you the code that is currently in the hivemq branch.

After these steps, you should be able to edit the source code of the microcontroller software and access the entire codebase.



## Committing changes

After making changes to the code, the following are the commands that you have to run in order to push or upload your code to the remote repository (the one on the github website).

The first thing we have to do is run:
```bash
git status
```
To check which branch we are currently in and what files are currently untracked (marked red).

After doing so, the next thing that we will do is add the untracked files to our tracking list by running the command:
```bash
git add .
```
NOTE: The command above assumes that you want to add all the altered files in the same tracking list.

After adding the edited files to the tracking list, we will now commit these changes. Committing is like scanning groceries over the counter.

```bash
git commit -m "<commit message>"
```

NOTE: Please refer to this guide by freecodecamp.org regarding good commit messages practice: https://www.freecodecamp.org/news/writing-good-commit-messages-a-practical-guide/

Once we have committed our changes and we want to upload our code to the remote repository, we will push it by running the following command:
```bash
git push -u origin <current-branch-name>
```
### Sample Scenario
I edited the main.cpp file and I want to upload the changes that I made. This is how I do it:
```bash
git status
```
After running the git status command, I found out that I am in the "hivemq" branch and that the file/s that I altered is/are main.cpp. 

The following are the chain of commands that I executed:

```bash
git add .
git commit -m "chore: added receivedCallback function to the software"
git push -u origin hivemq
```