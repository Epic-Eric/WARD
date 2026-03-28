export default function ConfirmDialog({ dialog, onResolve }) {
  if (!dialog) {
    return null;
  }

  return (
    <div className="confirm-dialog-backdrop" role="presentation" onClick={() => onResolve(false)}>
      <div
        className="confirm-dialog"
        role="dialog"
        aria-modal="true"
        aria-labelledby="confirm-dialog-title"
        onClick={(event) => event.stopPropagation()}
      >
        <p className="confirm-dialog__eyebrow">Confirm Action</p>
        <h2 id="confirm-dialog-title">{dialog.title}</h2>
        {dialog.detail ? <p className="confirm-dialog__detail">{dialog.detail}</p> : null}
        <div className="confirm-dialog__actions">
          <button type="button" className="confirm-dialog__button confirm-dialog__button--ghost" onClick={() => onResolve(false)}>
            {dialog.cancelLabel ?? "Cancel"}
          </button>
          <button
            type="button"
            className={`confirm-dialog__button confirm-dialog__button--${dialog.tone ?? "accent"}`}
            onClick={() => onResolve(true)}
          >
            {dialog.confirmLabel ?? "Confirm"}
          </button>
        </div>
      </div>
    </div>
  );
}
