# Duckievillage

![Duckievillage](https://raw.githubusercontent.com/RenatoGeh/duckievillage/master/duckieusp.png)

Duckievillage is an adaptation of [Duckietown](https://duckietown.org)'s OpenAI Gym
[gym-duckietown](https://github.com/duckietown/gym-duckietown) for the undergraduate level [Intro
to Robotics](https://uspdigital.usp.br/jupiterweb/obterDisciplina?sgldis=MAC0318&nomdis=) (MAC0318)
course held at the [Institute of Mathematics and Statistics](http://www.ime.usp.br/) (IME) of the
[University of São Paulo](https://www5.usp.br/#english) (USP).

This project is not to be thought of as a fork or standalone project, but as a complement to
Duckietown's Gym. To be more precise, Duckievillage's objectives are two-fold: to be an environment
wrapper around `DuckietownEnv`, which allows a more custom-made environment for our own robotics
course; and to be used as course walkthrough.

---

## Installation

The preferred way of installing Duckievillage is through the installation script. You'll need `git`
and Anaconda (or miniconda) installed. Simply run `install.sh` with your preferred shell. Pass, as
argument, your rcfile and you're good to go. For example, for zsh:

```bash
zsh install.sh ~/.zshrc
```

Or bash:

```bash
bash install.sh ~/.bashrc
```

Carefully read instructions. Once you're done, remember to activate the `duckietown` environment:

```bash
conda activate duckietown
```

You'll have to do this for every session before running Duckievillage.

### Manual installation

Alternatively, you might want to manually install by following these steps:

1. Install [Anaconda](https://www.anaconda.com/distribution/)
2. `git clone https:///gitlab.uspdigital.usp.br/mac0318-2021/duckievillage.git`
3. `cd duckievillage`
4. `git clone https://github.com/RenatoGeh/gym-duckietown.git duckietown`
5. `cd duckietown`
6. `conda env create -f environment.yaml --name duckietown`
7. `echo "export PYTHONPATH=\"\${PYTHONPATH}:$(pwd)\"" >> ~/.bashrc`
8. Re-source your rcfile (e.g. `source ~/.zshrc`)
9. `conda activate duckietown`
10. `pip install -r requirements.txt`
11. `cd ..`
12. `git clone https://gitlab.uspdigital.usp.br/mac0318-2021/assignments.git assignments`

Before every terminal session, activate `duckietown`'s environment:

```
conda activate duckietown
```

Test your environment:

```
python3 assignments/manual/manual.py
```

You should see the environment for the first exercise.

## Uninstallation

To uninstall, simply run `uninstall.sh` with the same shell you used for installing Duckievillage
and follow instructions.

---

## Updating Duckievillage

Before running Duckievillage, make sure you have the latest version by running `update.sh`:

```
zsh update.sh
```

This will update Duckievillage, Duckietown and assignments.

---

## Schedule

See the [wiki](https://gitlab.uspdigital.usp.br/groups/mac0318-2021/-/wikis/Vis%C3%A3o-geral).
