export default function AlertToast({ alert, onClose }) {
  if (!alert) {
    return null;
  }

  return (
    <div className={`alert-toast alert-toast--${alert.tone ?? "neutral"}`} role="status" aria-live="polite">
      <div className="alert-toast__body">
        <strong>{alert.title}</strong>
        {alert.detail ? <span>{alert.detail}</span> : null}
      </div>
      <button type="button" className="alert-toast__close" onClick={onClose} aria-label="Dismiss alert">
        ×
      </button>
    </div>
  );
}
