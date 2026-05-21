# Releasing the SDK

Git workflow and tag rules for `openmotion-sdk`. **Read this before creating
a tag** — the wrong shape breaks downstream app builds.

## Branch flow

- `main` — production. Only updated by merging an approved PR from `next`.
  Releases are cut from `main` (tag the merge commit), never directly from
  `next`.
- `next` — integration branch. Everything merges here first.
- `feature/<issue>-<short-desc>` — branch from `next`, PR back to `next`.

A release goes: `feature/* → next → main → tag → PyPI`. The
`next → main` PR is the release-approval gate; once merged, the tag on
`main` is what triggers the build + PyPI publish.

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

## Tag format — current policy

Only stable, suffix-free tags are accepted. Pre-release and any other
suffixed tags are **disallowed by policy** until further notice.

| Shape    | Example  | Triggers release pipeline |
|----------|----------|---------------------------|
| `X.Y.Z`  | `1.6.2`  | yes — stable              |
| `vX.Y.Z` | `v1.6.2` | yes — stable              |

**Do not use** anything with a suffix:

- `pre-1.6.2`  (previously allowed; now disallowed)
- `1.6.2-dev.1`
- `1.6.2-rc.1`
- `1.6.2-beta`
- `1.6.2.post1`

`pyproject.toml` still pins a permissive `setuptools_scm` regex
(`^(?:pre-)?v?(?P<version>\\d+\\.\\d+\\.\\d+)$`) and `release-build.yml`
will still build a `pre-*` tag if pushed — the restriction here is *policy*,
not enforcement. Don't push them.

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

1. Land all target commits on `next` via the usual `feature/* → next` PRs.
2. Open a PR `next → main` titled `Release: SDK next → main (...)`. This
   PR is the release-approval gate — reviewers sign off on the contents of
   the release here, not at tag time. Choose `X.Y.Z` for the eventual tag.
3. After the PR is merged, tag the merge commit on `main`:
   ```
   git checkout main
   git pull
   git tag -a 1.6.2 -m "release 1.6.2"
   git push origin 1.6.2
   ```
   The tag MUST point to a commit reachable from `main`. If you tagged a
   commit that's only on `next`, delete the tag and re-tag from `main`
   (see "If you tag wrong" below).
4. The tag push triggers `.github/workflows/release-build.yml`, which
   builds the wheel + sdist and creates a GitHub Release with the
   artifacts attached.
5. The GitHub Release `published` event triggers
   `.github/workflows/publish-pypi.yml`, which downloads the release
   assets and uploads them to PyPI via the OIDC trusted publisher.

The combination of (a) tag exists in the shape above and (b) the tagged
commit is reachable from `main` is what gates PyPI publication.

## Manual / test builds (no tag)

Use `workflow_dispatch` on `release-build.yml` with `pretend_version`
(e.g. `0.0.0-test1`) to produce a one-off build artifact without creating a
git tag. These artifacts are uploaded to the workflow run, not to a GitHub
Release, so they do not trigger the PyPI publish path.

## If you tag wrong

Delete the release + tag, then re-tag from the correct branch (`main`)
and push again:

```
gh release delete <tag> --repo OpenwaterHealth/openmotion-sdk --cleanup-tag --yes
git tag -d <tag>
git push origin :refs/tags/<tag>
```

Re-run any downstream builds that picked up the bad tag.
