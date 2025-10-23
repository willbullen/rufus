"""
Core Admin Utilities

Base admin classes and mixins for reuse.
"""

from django.contrib import admin


class ReadOnlyAdminMixin:
    """
    Mixin to make an admin interface read-only.
    """
    
    def has_add_permission(self, request):
        return False
    
    def has_change_permission(self, request, obj=None):
        return False
    
    def has_delete_permission(self, request, obj=None):
        return False


class TimeStampedAdminMixin:
    """
    Mixin to add timestamp fields to admin readonly fields.
    """
    
    def get_readonly_fields(self, request, obj=None):
        readonly_fields = list(super().get_readonly_fields(request, obj))
        if 'created_at' not in readonly_fields:
            readonly_fields.append('created_at')
        if 'updated_at' not in readonly_fields:
            readonly_fields.append('updated_at')
        return readonly_fields

