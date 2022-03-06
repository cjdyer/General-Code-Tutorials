# General Code Tutorials

This repository aims to give a base level understanding of important coding practices and features we have not been taught at University.

## Table of Contents
1. [Getting started](#getting-started)
2. [Understanding Git Terminology](#understanding-git-terminology)
3. [Using Git](#using-git)

## Getting started 

The best place to start with this repository is reading through this file first and then moving to the C++ folder and following through the README located there as well. I recommend starting with the C++ folder as it is important to understand C++ before you move into ROS, as you may not understand some of the concepts used. Thereby, misunderstanding a ROS feature for that provided by C++. It also a good idea to try and run the provided examples yourself, along with changing small components to check your understanding. I often add my own comments to a file from an unknown repository, as this helps me follow the logical flow.

## Understanding Git Terminology

To get a quick understand I will explain a few key terms.

### Git

Git in itself is just a version control software. It can be used to track and sort different versions of code across your own machine and others machines. Git as a program is only a local process, meaning it is run on your computer, without context of the global (everyone elses computers) state of the project. This can be solved by linking Git with a website like Github or gitlab.

### Repository

A repository is the name given to a git project. A repository contains all aspects of the project, its files, the git history and any branches. The name of this repository is [General-Code-Tutorials](https://github.com/cjdyer/General-Code-Tutorials).

### Branch

Branches can be created inside of your repository at any point, these all exist as part of the repository. When a repository is created, a `master` branch is created, this branch should not be deleted and should be used as the base for which all of your other branches stem from. A branch is generally used as an alternate build of the project, or as a start of working on an experimental feature. When working in teams, it is good practice to create a branch when you start making changes. Once you have made your changes and implemented the feature you need, create a `pull request` and `merge` the changes back into the `master` branch. It is generally a good idea to focus on changing one feature of the project at a time, or even more ideally, just editing one file.

### Commit

A commit is a series of changes packaged into one object. Commits exist as a difference between the previous state of the repository and the current state, where additions will be typically denoted in green and removals denoted in red. It is generally a good idea to keep commits small and exact when working on larger projects, that way you can track changes to the system without having to dig through each commit one by one. A commit will exist as part of a chnage on the current active branch, and does not carry over to all branches until a `pull request` is created and `merged`.

### Pull Request

A pull request can be created after a new branch is created and changes have been committed. It is called a pull request, as it is a request to pull the new code into the main/master branch. Most of the time a pull request can be automatically merged, as none of the files you have edited have been changed since you created the branch. If the pull request cannot be automatically merged, you will have to go through each conflicting file and audit the code to see what code to keep and what code to remove. If the previously noted processes are followed, then a pull request should be an easy and seemless event.

## Using Git

If you are using Linux or MacOS then git should automatically be installed on the system; for Windows users, you will need to [install git](https://git-scm.com/download/win). To check git is instlalled you can use the command:

>`git --version`

The easiest place to start is to clone this repository. First find a folder you would like the repository to be downloaded to, and open a terminal at the folder. For windows you can type `cmd` into the file explorer like [this](https://www.youtube.com/watch?v=CDSxs8Mt9zU). You can then use the following command.

>`git clone https://github.com/cjdyer/General-Code-Tutorials.git`

This will create a local version of the repository. Where you can make edits that do not affect the repository found online.

To create your own git repository the easiest way is to create a use Github and then clone that repository, this will ensure you are explictly linked to the repository that is found online.

It is recommended that [Github Desktop](https://desktop.github.com/) or [Visual Studio Code](https://code.visualstudio.com/) are used to manage all git usage whilst still learning how to use git. I would recommend using Github Desktop if you more of a visual person, to start with, then move to VSCode when you are more confident.

### Using Visual Studio Code Git

After installing VSCode, install the [Github extesion](https://marketplace.visualstudio.com/items?itemName=GitHub.vscode-pull-request-github) to link with Github.

Once you have opened a folder (workspace) with a git repository, you will be able to access VSCodes source control features. The main place these features are located are on the left-most menu or by using `ctrl+shift+G`. Make a change to a file, and you will see it appears in a list on this tab. 

To practice commiting changes, and a commit comment to the text box at the top (contains the text *Message (Ctril+Enter to commit on 'master')*). Then click the tick button above this text box. If you are presented with the text saying something along the lines of 'no staged files', click **always**. Then click sync changes.