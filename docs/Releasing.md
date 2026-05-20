# Releasing the SDK

Git workflow and tag rules for `openmotion-sdk`. **Read this before creating
a tag** — the wrong shape breaks downstream app builds.

## Branch flow

- `main` — production. Only updated by promoting `next` after a release.
- `next` — integration branch. Everything merges here first.
- `feature/<issue>-<short-desc>` — branch from `next`, PR back to `next`.

## How downstream apps consume the SDK

The app workflows (`openmotion-bloodflow-app`, `openmotion-test-app`) pick
their SDK source based on the *app's* tag shape:

| App tag           | SDK source                                                          |
|-------------------|---------------------------------------------------------------------|
| `X.Y.Z`           | latest published `openmotion-sdk` wheel from PyPI / GitHub Releases |
| `X.Y.Z-rc.N`      | latest published `openmotion-sdk` wheel from PyPI / GitHub Releases |
| `X.Y.Z-dev.N`     | `pip install git+https://...openmotion-sdk.git@next` (source build) |

The third row is the critical one: dev app builds **build the SDK from source
off `next`**, so `next` must always be installable. That means
`setuptools_scm` has to successfully parse the most recent reachable tag.

## Tag format — must match the `setuptools_scm` regex

`pyproject.toml` pins:

```toml
[tool.setuptools_scm]
tag_regex = "^(?:pre-)?v?(?P<version>\\d+\\.\\d+\\.\\d+)$"
```

Only these tag shapes are valid on this repo:

| Shape       | Example     | Triggers release pipeline |
|-------------|-------------|---------------------------|
| `X.Y.Z`     | `1.6.1`     | yes — stable              |
| `vX.Y.Z`    | `v1.6.1`    | yes — stable              |
| `pre-X.Y.Z` | `pre-1.6.2` | yes — pre-release (rc0)   |

**Do not use** anything with a suffix after the three numbers:

- `1.6.2-dev.1`
- `1.6.2-rc.1`
- `1.6.2-beta`
- `1.6.2.post1`

### Why suffixed tags break things

When `pip install git+...@next` runs anywhere downstream, `setuptools_scm`
calls `git describe`, gets the most recent reachable tag, and runs it through
`tag_regex`. A non-matching tag returns `None`, which trips
`assert version is not None` in `setuptools_scm._scm_version._parse_tag` and
kills the wheel build with `AssertionError`.

The SDK's own `release-build.yml` sidesteps this by setting
`SETUPTOOLS_SCM_PRETEND_VERSION` from the tag name — so the SDK *can* publish
a wheel for a bad-shaped tag. That gives a false sense of safety: the SDK
release succeeds, but every downstream source-install from `@next`
immediately after will fail until the bad tag is removed.

**Past incident:** tag `1.6.2-dev.1` on `next` broke `openmotion-bloodflow-app`
build `1.1.1-dev.6`
([run 26131915045](https://github.com/OpenwaterHealth/openmotion-bloodflow-app/actions/runs/26131915045)).
The wheel published fine on the SDK side; every consumer broke.

## Cutting a release

1. Land all target commits on `next`.
2. Choose `X.Y.Z` (stable) or `pre-X.Y.Z` (RC).
3. From `next`:
   ```
   git tag -a 1.6.2 -m "release 1.6.2"
   git push origin 1.6.2
   ```
4. `.github/workflows/release-build.yml` builds wheel + sdist and attaches
   them to a GitHub release.
5. PyPI publishing handled by `publish-pypi.yml`.
6. Promote `next` → `main` after the release stabilizes.

## Manual / test builds (no tag)

Use `workflow_dispatch` on `release-build.yml` with `pretend_version`
(e.g. `0.0.0-test1`) to produce a one-off build artifact without creating a
git tag.

## If you tag wrong

Delete the release + tag, then re-run the downstream build:

```
gh release delete <tag> --repo OpenwaterHealth/openmotion-sdk --cleanup-tag --yes
git tag -d <tag>
```
