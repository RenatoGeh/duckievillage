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

### Alternative `pip` installation

Another alternative is to install everything through pip. It is **highly recommended** you use a
python environment if you install this way. Here is an example on how to create one:

`mkdir -p ~/.venv && cd ~/.venv && python3 -m venv duckietown && cd -`

To activate this environment, do

`source ~/.venv/duckietown/bin/activate`

You can then install Duckievillage by the following command under the Duckievillage root directory:

`pip install -r requirements.txt`

Clone Duckietown if you haven't yet:

`[ ! -d "duckietown" ] && git clone https://github.com/RenatoGeh/gym-duckietown.git duckietown`

And assignments:

`[ ! -d "assignments" ] && git clone https://gitlab.uspdigital.usp.br/mac0318-2021/assignments.git assignments`

Make sure your rcfile (replace `.bashrc` with your own) contains Duckietown in your python path:

`echo "export PYTHONPATH=\"\${PYTHONPATH}:$(pwd)/duckietown/src\"" >> ~/.bashrc`


Resource your rcfile:

`source ~/.bashrc`

Test your setup with:

```
python3 assignments/manual/manual.py
```

Don't forget to activate your environment for every new shell session!

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

## Frequently Asked Questions

1. Why do I keep getting permission denied when trying to clone Duckievillage by HTTPS?

> You have to set up an HTTPS password: https://gitlab.uspdigital.usp.br/-/profile/password/edit

2. I get a permission denied when trying to clone from within a WSL shell!

> You should clone from a WSL partition, and not your Windows NTFS. Do `cd ~` and retry.

3. `ModuleNotFoundError: No module named 'duckievillage'`

> You have to `cd` to the Duckievillage root directory (i.e. the directory you cloned). If the
> error persists, try either closing your shell session and opening another one (don't forget to
> `cd` to the Duckievillage root directory and activate your environment with `conda`), or sourcing
> your rcfile.

4. `ModuleNotFoundError: No module named 'zuper_commons'`

> You have to activate the Duckietown environment: `conda activate duckietown`.

5. `install.sh: line 2: syntax error near unexpected token $'{\r`

> This is a Windows error (due to carriage return). Git Windows adds carriage returns. You can
> Removing these carriage returns or cloning within WSL should fix this. Do note that Windows is
> not supported in Duckievillage.

6. `pyglet.gl.ContextException: Could not create GL context`

> This might either be a VM issue or a Pyglet version issue. VMs are not supported in
> Duckievillage. If it's Pyglet, you may want to either try updating the Anaconda environment with
> the latest python and packages; or alternatively ditch Anaconda and install everything with
> `pip`. For the latter, you might want to completely remove Anaconda so Python doesn't get
> confused with package versions (or at least completely isolate Anaconda from another Python
> environment. See the [pip alternative](#alternative-pip-installation) instructions. Wayland might
> also cause this problem. Consider using a Xorg backend for your Desktop Environment or Window
> Manager instead.

---

## Schedule

See the [wiki](https://gitlab.uspdigital.usp.br/groups/mac0318-2021/-/wikis/Vis%C3%A3o-geral).
