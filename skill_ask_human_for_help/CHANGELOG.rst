^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package skill_ask_human_for_help
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-06-18)
------------------
* enable copyright tests + update license file to pass the test
* remove dep on archlint, as the skill manifest is not here anymore
* minor tuning
* Remove timer for detecting human
* spin_all for hri listener
* spinning manually hri listener
* update readme
* add in package.xml that it implements the ask_human_for_help skill
* lint fix
* move to skill namespace and hri listener issues debugging
* fix bug appearing when ask skill is not available
* use navigation skill to go to pose
* code cleanup
* delete unused config and updated package.xml dependencies
* fix lint
* Bug fixing and improvements from testing on the robot
* using ask skill and waiting for a human to be visible
* Contributors: Séverin Lemaignan, ferrangebelli

0.2.1 (2025-06-18)
------------------
* linting
* {skillint -> archlint}
* Improvements and debugging to have first working version on the robot
* Contributors: Séverin Lemaignan, ferrangebelli

0.2.0 (2025-02-17)
------------------
* linting
* {ask_human_for_help -> skill_ask_human_for_help}
* move message to interaction_skills repo + remove node to root folder
* rename to ask human for help
* ask for help with printing
* Initial commit
* Contributors: Sara Cooper, Séverin Lemaignan, saracooper
