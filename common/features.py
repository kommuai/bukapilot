from openpilot.common.params import Params

FEATURE_DELIMITER = ', '

FEATURES = {
  "ignore-dm",
  "clear-code",
}

_params_instance: Params | None = None

def _get_params() -> Params:
  """Lazily returns a shared Params instance."""
  global _params_instance
  if _params_instance is None:
    _params_instance = Params()
  return _params_instance

def _process_feature_string(feature_string_input: str) -> str:
  """Normalises and validates a feature string, preserving order."""
  cleaned_input = (feature_string_input or "").lower()
  ordered_valid_features = []
  processed_features = set()
  for feature_part in cleaned_input.split(','):
    feature = feature_part.strip()
    if feature and feature in FEATURES and feature not in processed_features:
      ordered_valid_features.append(feature)
      processed_features.add(feature)
  return FEATURE_DELIMITER.join(ordered_valid_features)

def _get_features_param() -> str:
  """Safely retrieves the feature parameter as a string."""
  return (_get_params().get("FeaturesPackage") or b"").decode()

def _put_features_param(value: str) -> None:
  """Puts the feature parameter value."""
  _get_params().put_nonblocking("FeaturesPackage", value)

class Features:
  def set_features(self, feature_string: str) -> None:
    """Sets the feature parameter."""
    value_to_store = _process_feature_string(feature_string)
    _put_features_param(value_to_store)

  def has(self, feature_name: str) -> bool:
    """Checks if a feature is active."""
    active: set[str] = set(filter(None, _get_features_param().split(FEATURE_DELIMITER)))
    return feature_name.lower() in active

  def validate_and_clean_features(self) -> None:
    """Validates and cleans existing features on device startup."""
    current_features_raw = _get_features_param()
    cleaned_features_string = _process_feature_string(current_features_raw)
    if cleaned_features_string != current_features_raw:
      _put_features_param(cleaned_features_string)
