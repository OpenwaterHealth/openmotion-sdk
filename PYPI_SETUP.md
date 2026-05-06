# PyPI publishing setup

Guide for publishing `openmotion-sdk` to PyPI, including the
one-time fix needed after the `openmotion-pylib` → `openmotion-sdk`
rename in commit `0f7bb52`.

## Current state (as of 1.5.5)

- `pyproject.toml` declares `name = "openmotion-sdk"`.
- `python -m build` produces `openmotion_sdk-X.Y.Z-py3-none-any.whl`
  and `openmotion_sdk-X.Y.Z.tar.gz`.
- PyPI has the **old** project `openmotion-pylib` (last published 1.4.5).
- PyPI does **not** have `openmotion-sdk` registered yet.
- `.github/workflows/publish-pypi.yml` uses Trusted Publishing (OIDC)
  via `pypa/gh-action-pypi-publish`. OIDC publishers can publish *to*
  existing projects but **cannot create new projects**.
- Result: every `1.5.x` publish run since the rename has failed with
  `400 Non-user identities cannot create new projects.`

The new preflight step in `publish-pypi.yml` now catches this case and
prints the exact fix in the GitHub Actions run summary instead of
letting twine fail with a confusing message.

## One-time fix (required to publish 1.5.x and onward)

Pick ONE of these.

### Option A — Pending Publisher (recommended)

PyPI lets you pre-authorize an OIDC publisher to create a project on
its first upload.

1. Sign in at <https://pypi.org/account/login/>.
2. Go to <https://pypi.org/manage/account/publishing/> and scroll to
   "Add a new pending publisher".
3. Fill in:
   - **PyPI Project Name:** `openmotion-sdk`
   - **Owner:** `OpenwaterHealth`
   - **Repository name:** `openmotion-sdk`
   - **Workflow name:** `publish-pypi.yml`
   - **Environment name:** *(leave blank unless your workflow sets one)*
4. Save.
5. Trigger the publish workflow:
   - Either retag and push (creates a new GitHub Release → workflow
     fires automatically), or
   - `workflow_dispatch` with `tag` set to the existing tag (e.g.
     `1.5.5`) — see the SDK Actions tab.
6. The first run will succeed and register `openmotion-sdk` on PyPI.
   Subsequent runs use the same publisher, so no further setup needed.

### Option B — One-time manual upload

If you can't add a Pending Publisher, do a single manual upload from
a developer machine. This registers the project on PyPI under your
PyPI user account, after which OIDC is authorized to publish updates.

```bash
git checkout 1.5.5            # or whichever tag you want as the seed
python -m pip install --upgrade build twine
python -m build
twine upload dist/openmotion_sdk-1.5.5*.{whl,tar.gz}
# enter your __token__ + a user-scoped PyPI API token
```

After the project exists, update the OIDC publisher on PyPI to point
at `openmotion-sdk`, then re-trigger the workflow for any later tags.

## What about `openmotion-pylib`?

The old PyPI project still exists with versions up to 1.4.5. Decide
how to retire it:

### Recommended: deprecate-and-redirect

1. After `openmotion-sdk` is publishing successfully, mark
   `openmotion-pylib` as deprecated:
   - <https://pypi.org/project/openmotion-pylib/> → "Manage" →
     edit description, prepend a deprecation banner pointing to
     `openmotion-sdk`.
2. Optionally publish one final `openmotion-pylib` version
   (e.g. `1.4.6`) whose `pyproject.toml` is just:
   ```toml
   [project]
   name = "openmotion-pylib"
   version = "1.4.6"
   description = "Renamed to openmotion-sdk; please install that instead."
   dependencies = ["openmotion-sdk"]
   ```
   …and a stub `__init__.py` that emits a `DeprecationWarning`
   on import. Pip-installing the old name then transparently
   pulls in the new package.

### Alternative: leave alone

Just stop pushing to `openmotion-pylib`. Keep the entry alive for
historical references; let the description on PyPI mention the new
name.

## Verifying the fix

```bash
# Should return 200 once the new project exists
curl -s -o /dev/null -w "%{http_code}\n" https://pypi.org/pypi/openmotion-sdk/json

# Should return the published version after a successful run
pip index versions openmotion-sdk
```

## Downstream impact

Until `openmotion-sdk` is on PyPI, the **bloodflow app's release
build** will fail for any `X.Y.Z` (production) or `X.Y.Z-rc.N` (release
candidate) tag — those routes do `pip install --upgrade openmotion-sdk`
(see `openmotion-bloodflow-app/AGENTS.md`). The current in-flight tag
`1.0.4-rc.0` (pushed 2026-04-29) is one such build.

`X.Y.Z-dev.N` tags are unaffected — they install from
`openmotion-sdk@next` source directly.
