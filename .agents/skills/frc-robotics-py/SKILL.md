---
name: frc-robotics-py
description: Use when planning/developing/debugging robotpy related projects. Use when making a FRC robot in python.
license: MIT
metadata:
  author: Randy Z
  version: "1.0.0"
---

# Best Practice Developing a FRC Robot in Python

FRC programming is not very common so the available information is limited and easily outdated. You should primarily focus on reliable sources mentioned in this skill.

## 1. Where to Find Documentation

### Context7: Use it for wpilib but NOT RobotPy
[WPILIB](https://docs.wpilib.org/en/stable/) context7 id: `wpilib_en_stable`
Use it to get general informaiton about FRC control system. For example
- "What's Command-Based Programming?"
- "How to code for the Power Distribution Module?"

However, Context7 **don't** have the main RobotPy docs. Do not spend time searching Context7 for `robotpy` or vendor API details.

### Primary sources (use these)

Documentations
| Topic | URL |
|-------|-----|
| RobotPy detailed docs | https://robotpy.readthedocs.io/en/stable/index.html |
| RobotPy REV API | https://robotpy.readthedocs.io/projects/rev/en/stable/api.html |
| REV class list | https://robotpy.readthedocs.io/projects/rev/en/stable/rev.html |
| Navx API | https://robotpy.readthedocs.io/projects/navx/en/stable/api.html |
| Phoenix 6 API | https://api.ctr-electronics.com/phoenix6/stable/python/ |
| WPILib Python docs | https://docs.wpilib.org/en/stable/docs/software/python/ |

Example Projects
- https://github.com/robotpy/mostrobotpy/tree/main/examples

### How to read them quickly
- Use `websearch_web_search_exa` for "how do I do X in RobotPy 2026" style questions.
- Use `webfetch` with `format=markdown` to pull specific API pages when you know the class name (e.g. `rev/SparkMax.html`).
- Search GitHub with `grep_app_searchGitHub` only as a fallback; RobotPy 2026 code examples are still sparse.

### Other Information Refernces
- [Quick intro to the competition](./references/first-robotics-competition.md)
- [How does code run on actual robot](./references/robot-architecture.md)


## 2. Trust Running Code Over Memory

RobotPy 2026 has several API changes from earlier versions and from Java WPILib. **Always verify by actually importing and running**:

In virtual environment,
```powershell
python -c "import rev; print([x for x in dir(rev) if 'Reset' in x])"
python -m robotpy test
python -m robotpy sim --nogui
```

If a class/constant feels off, inspect it directly:

```python
import wpilib
print([x for x in dir(wpilib) if 'MotorController' in x])
```

## 3. Utilize AGENTS.md

RobotPy projects have less well-known structures and team specific conventions. These are usually written in `AGENTS.md`.
- If the project doesn't have `AGENTS.md` yet, inform the user first, and then create one follow [this](./references/create-agents-md.md).
- If you plan to make big changes, read AGENTS.md first.
- If you made big changes, update AGENTS.md if necessary


## 4. Code Comment

People usually have limited knowledge about various FRC stuff, so comments are important.
- Write brief doc string for new methods.
- Write comments for obsure calculations or professional concept (e.g. PID, kinematics, odometry)
- Write brief comments for each step of a processing or a setup. 

## 5. Suggested Workflow

1. Read the relevant file in this repo.
2. If you need API details, `webfetch` the specific RobotPy/WPILib page.
3. Make the smallest change that could work.
4. Run `python -m robotpy test` immediately.
5. If tests pass, run `python -m robotpy sim --nogui` for 10–20 seconds.
6. Update this guide if you discover a new gotcha.
